#include <string>
#include <sstream>
#include <memory>
#include <chrono>
#include <vector>
#include <array>
#include <utility>
#include <format>
#include <functional>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include <opencv2/core/quaternion.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>


namespace util {

	template <typename T>
	struct identity { typedef T type; };

	template <typename T>
	void declare_param(rclcpp::Node* node, const std::string param_name, T& param, const typename identity<T>::type& default_value) {
		node->declare_parameter(param_name, default_value);
		node->get_parameter(param_name, param);
	}

}


class ArucoServer : public rclcpp::Node
{
public:
	ArucoServer();

protected:
	void publish_debug_frame();

	struct ImageSource
	{
		ArucoServer* ref;

		image_transport::Subscriber image_sub;
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;

		cv::Mat last_frame;
		cv::Mat1d
			calibration = cv::Mat1d::zeros(3, 3),
			// undistorted_calib = cv::Mat1f::zeros(3, 3),
			distortion = cv::Mat1d::zeros(1, 5);

		bool valid_calib_data = false;

		void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img);
		void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);
	};

	struct TagDescription
	{
		using Ptr = std::shared_ptr<TagDescription>;
		using ConstPtr = std::shared_ptr<const TagDescription>;

		std::array<cv::Point3f, 4>
			world_corners,
			rel_corners;
		
		union
		{
			struct{ double x, y, z; };
			double translation[3];
		};
		union
		{
			struct{ double qw, qx, qy, qz; };
			double rotation[4];
		};
		union
		{
			struct{ double a, b, c, d; };
			double plane[4];
		};

		static Ptr fromRaw(const std::vector<double>& world_corner_pts);
	};

private:
	std::unordered_map<int, TagDescription::ConstPtr> obj_tag_corners;
	std::vector<ImageSource> sources;

	image_transport::ImageTransport img_tp;
	image_transport::Publisher debug_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;

	tf2_ros::Buffer tfbuffer;
	tf2_ros::TransformListener tflistener;
	tf2_ros::TransformBroadcaster tfbroadcaster;

	std::string tags_frame_id, base_frame_id;
	bool filter_prev_proximity, filter_bbox;
	cv::Point3d bbox_min, bbox_max;

	std::chrono::system_clock::time_point last_publish;
	std::chrono::duration<double> target_pub_duration;

	cv::Ptr<cv::aruco::Dictionary> aruco_dict;
	cv::Ptr<cv::aruco::DetectorParameters> aruco_params;
	struct
	{
		std::vector<std::vector<cv::Point2f>> tag_corners;
		std::vector<int32_t> tag_ids;
		std::vector<cv::Point2f> img_points;
		std::vector<cv::Point3f> obj_points;
		std::vector<cv::Vec3d> tvecs, rvecs;
		std::vector<double> eerrors;
	} _detect;

};





ArucoServer::ArucoServer():
	Node{ "aruco_server" },
	img_tp{ std::shared_ptr<ArucoServer>(this, [](auto*){}) },
	tfbuffer{ std::make_shared<rclcpp::Clock>(RCL_ROS_TIME) },
	tflistener{ tfbuffer },
	tfbroadcaster{ *this },
	last_publish{ std::chrono::system_clock::now() }
{
	std::vector<std::string> img_topics, info_topics;
	util::declare_param<std::vector<std::string>>(this, "img_topics", img_topics, {});
	util::declare_param<std::vector<std::string>>(this, "info_topics", info_topics, {});

	std::string dbg_topic;
	util::declare_param(this, "debug_topic", dbg_topic, "aruco_server/debug/image");
	this->debug_pub = this->img_tp.advertise(dbg_topic, 1);

	int pub_freq;
	util::declare_param(this, "debug_pub_freq", pub_freq, 30);
	this->target_pub_duration = std::chrono::duration<double>{ 1. / pub_freq };

	const size_t
		n_img_t = img_topics.size(),
		n_info_t = info_topics.size();

	if(n_img_t < 1 || n_info_t < n_img_t)
	{
		RCLCPP_ERROR(this->get_logger(), "No image topics provided or mismatched image/info topics");
		return;	// exit?
	}

	util::declare_param(this, "tags_frame_id", this->tags_frame_id, "world");
	util::declare_param(this, "base_frame_id", this->base_frame_id, "base_link");

	std::string pose_topic;
	util::declare_param(this, "pose_pub_topic", pose_topic, "/aruco_server/pose");
	this->pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 10);

	util::declare_param(this, "filtering/prev_transform", this->filter_prev_proximity, false);
	util::declare_param(this, "filtering/bounding_box", this->filter_bbox, false);
	std::vector<double> param_buff;
	util::declare_param(this, "filtering/bounds_min", param_buff, {0., 0., 0.});
	if(param_buff.size() > 2)
		memcpy(&this->bbox_min, param_buff.data(), sizeof(cv::Point3d));
	util::declare_param(this, "filtering/bounds_max", param_buff, {0., 0., 0.});
	if(param_buff.size() > 2)
		memcpy(&this->bbox_max, param_buff.data(), sizeof(cv::Point3d));

	int aruco_dict_id = cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h11;
	util::declare_param(this, "aruco_predefined_family_idx", aruco_dict_id, aruco_dict_id);

	// TODO: expose detector params

	this->aruco_dict = cv::aruco::getPredefinedDictionary(aruco_dict_id);
	this->aruco_params = cv::aruco::DetectorParameters::create();

	std::vector<double> tag_ids;
	util::declare_param(this, "tag_ids", tag_ids, {});

	const size_t n_tags = tag_ids.size();
	if(n_tags < 1)
	{
		RCLCPP_ERROR(this->get_logger(), "No tag ids enabled!");
		return;	// exit?
	}

	for(size_t i = 0; i < n_tags; i++)
	{
		param_buff.clear();
		const int id = static_cast<int>(tag_ids[i]);

		std::stringstream topic;
		topic << "tag" << id;
		util::declare_param(this, topic.str(), param_buff, {});

		auto ptr = this->obj_tag_corners.insert({ id, TagDescription::fromRaw(param_buff) });
		if(ptr.second && ptr.first->second)	// successful insertion and successful parse
		{
			auto desc = ptr.first->second;

			RCLCPP_INFO(this->get_logger(),
				"Successfully added description for tag %d :\n"
				"Corners -- [\n\t(%f, %f, %f),\n\t(%f, %f, %f),\n\t(%f, %f, %f),\n\t(%f, %f, %f)\n]\n"
				"Pose -- (%f, %f, %f) ~ [%f, %f, %f, %f]",
				id,
				desc->world_corners[0].x,
				desc->world_corners[0].y,
				desc->world_corners[0].z,
				desc->world_corners[1].x,
				desc->world_corners[1].y,
				desc->world_corners[1].z,
				desc->world_corners[2].x,
				desc->world_corners[2].y,
				desc->world_corners[2].z,
				desc->world_corners[3].x,
				desc->world_corners[3].y,
				desc->world_corners[3].z,
				desc->x,
				desc->y,
				desc->z,
				desc->qw,
				desc->qx,
				desc->qy,
				desc->qz);
		}
		else
		{
			this->obj_tag_corners.erase(id);	// remove since invalid
			RCLCPP_ERROR(this->get_logger(), "Failed to setup description for tag %d", id);
		}
	}

	this->sources.resize(n_img_t);
	for(size_t i = 0; i < n_img_t; i++)
	{
		RCLCPP_DEBUG(this->get_logger(), "Source %ld using image topic [%s] and info topic [%s]", i, img_topics[i].c_str(), info_topics[i].c_str());

		this->sources[i].ref = this;
		this->sources[i].image_sub =
			this->img_tp.subscribe( img_topics[i], 1,
				std::bind( &ArucoServer::ImageSource::img_callback, &this->sources[i], std::placeholders::_1 ) );
		this->sources[i].info_sub =
			this->create_subscription<sensor_msgs::msg::CameraInfo>( info_topics[i], 10,
				std::bind( &ArucoServer::ImageSource::info_callback, &this->sources[i], std::placeholders::_1 ) );
	}
}

void ArucoServer::publish_debug_frame()
{
	std::vector<cv::Mat> frames;
	frames.reserve(this->sources.size());
	for(const ImageSource& s : this->sources)
	{
		if(s.last_frame.size().area() > 0)
		{
			frames.insert(frames.end(), s.last_frame);
		}
	}

	if(frames.size() > 0)
	{
		try {
			cv::Mat pub;
			cv::hconcat(frames, pub);

			std_msgs::msg::Header hdr;
			hdr.frame_id = this->base_frame_id;
			hdr.stamp = this->get_clock()->now();
			this->debug_pub.publish(cv_bridge::CvImage(hdr, "bgr8", pub).toImageMsg());
		}
		catch(const std::exception& e)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to publish debug image concatenation: %s", e.what());
		}
	}
}

void ArucoServer::ImageSource::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img)
{
	if(!this->valid_calib_data) return;

	cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvCopy(*img, "mono8");
	cv::cvtColor(cv_img->image, this->last_frame, CV_GRAY2BGR);

	ref->_detect.tag_corners.clear();
	ref->_detect.tag_ids.clear();

	// cv::Mat undistorted;
	// try {
	// 	cv::undistort(
	// 		cv_img->image,
	// 		undistorted,
	// 		this->calibration,
	// 		this->distortion);
	// }
	// catch(const std::exception& e)
	// {
	// 	RCLCPP_ERROR(ref->get_logger(), "Error undistorting image: %s", e.what());
	// 	// detect markers with original image...
	// 	undistorted = cv_img->image;
	// }

	try {
		cv::aruco::detectMarkers(
			cv_img->image,
			ref->aruco_dict,
			ref->_detect.tag_corners,
			ref->_detect.tag_ids,
			ref->aruco_params);
	}
	catch(const std::exception& e)
	{
		RCLCPP_ERROR(ref->get_logger(), "Error detecting markers: %s", e.what());
	}

	cv::aruco::drawDetectedMarkers(this->last_frame, ref->_detect.tag_corners, ref->_detect.tag_ids, cv::Scalar{0, 255, 0});

	if(const size_t n_detected = ref->_detect.tag_ids.size(); (n_detected > 0 && n_detected == ref->_detect.tag_corners.size()))	// valid detection(s)
	{
		ref->_detect.rvecs.clear();
		ref->_detect.tvecs.clear();
		ref->_detect.eerrors.clear();

		ref->_detect.obj_points.clear();
		ref->_detect.obj_points.reserve(n_detected * 4);
		ref->_detect.img_points.clear();
		ref->_detect.img_points.reserve(n_detected * 4);

		size_t matches = 0;
		bool all_coplanar = true;
		cv::Vec4d _plane = cv::Vec4d::zeros();
		TagDescription::ConstPtr primary_desc;

		for(size_t i = 0; i < n_detected; i++)
		{
			auto search = ref->obj_tag_corners.find(ref->_detect.tag_ids[i]);
			if(search != ref->obj_tag_corners.end())
			{
				auto& result = search->second;
				auto& obj_corners = result->world_corners;
				auto& rel_obj_corners = result->rel_corners;
				auto& img_corners = ref->_detect.tag_corners[i];

				ref->_detect.obj_points.insert(ref->_detect.obj_points.end(), obj_corners.begin(), obj_corners.end());
				ref->_detect.img_points.insert(ref->_detect.img_points.end(), img_corners.begin(), img_corners.end());

				if(++matches == 1) primary_desc = result;	// first match

				// for(size_t x = 0; x < obj_corners.size(); x++)
				// {
				// 	cv::putText(this->last_frame, std::format("({}, {}, {})", obj_corners[x].x, obj_corners[x].y, obj_corners[x].z), static_cast<cv::Point>(img_corners[x]), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(255, 100, 0), 1, cv::LINE_AA);
				// }

				cv::solvePnPGeneric(
					rel_obj_corners,
					img_corners,
					this->calibration,
					this->distortion,
					ref->_detect.rvecs,
					ref->_detect.tvecs,
					false,
					cv::SOLVEPNP_IPPE_SQUARE,
					cv::noArray(),
					cv::noArray(),
					ref->_detect.eerrors);

				// cv::drawFrameAxes(this->last_frame, this->calibration, this->distortion, _rvec, _tvec, 0.2f, 3);

				for(size_t s = 0; s < ref->_detect.rvecs.size(); s++)
				{
					cv::Vec3d
						&_rvec = ref->_detect.rvecs[s],
						&_tvec = ref->_detect.tvecs[s];

					cv::Quatd q = cv::Quatd::createFromRvec(_rvec);

					geometry_msgs::msg::TransformStamped tfs;
					tfs.header = cv_img->header;
					tfs.child_frame_id = std::format("tag_{}_s{}", ref->_detect.tag_ids[i], s);
					tfs.transform.translation = reinterpret_cast<geometry_msgs::msg::Vector3&>(_tvec);
					tfs.transform.rotation.w = q.w;
					tfs.transform.rotation.x = q.x;
					tfs.transform.rotation.y = q.y;
					tfs.transform.rotation.z = q.z;

					ref->tfbroadcaster.sendTransform(tfs);
				}

				if(all_coplanar)
				{
					const cv::Vec4d& _p = reinterpret_cast<const cv::Vec4d&>(result->plane);
					if(matches == 1) _plane = _p;
					else if(1.0 - std::abs(_plane.dot(_p)) > 1e-6) all_coplanar = false;
				}
			}
		}

		// RCLCPP_INFO(ref->get_logger(), "MEGATAG solve params -- obj points: %lu, img points: %lu", ref->_detect.obj_points.size(), ref->_detect.img_points.size());

		if(matches == 1)	// reuse debug tvecs and rvecs ^^
		{
			Eigen::Isometry3d tf = ( Eigen::Quaterniond{ primary_desc->qw, primary_desc->qx, primary_desc->qy, primary_desc->qz } *
				Eigen::Translation3d{ -reinterpret_cast<const Eigen::Vector3d&>(primary_desc->translation) } );

			for(size_t i = 0; i < ref->_detect.tvecs.size(); i++)
			{
				cv::Quatd r = cv::Quatd::createFromRvec(ref->_detect.rvecs[i]);
				Eigen::Isometry3d _tf = reinterpret_cast<Eigen::Translation3d&>(ref->_detect.tvecs[i]) * (Eigen::Quaterniond{ r.w, r.x, r.y, r.z } * tf);

				Eigen::Vector3d _t;
				_t = _tf.translation();
				ref->_detect.tvecs[i] = reinterpret_cast<cv::Vec3d&>(_t);

				Eigen::Quaterniond _r;
				_r = _tf.rotation();
				ref->_detect.rvecs[i] = cv::Quatd{ _r.w(), _r.x(), _r.y(), _r.z() }.toRotVec();
			}

			RCLCPP_INFO(ref->get_logger(), "SolvePnP successfully yielded %lu solution(s) using 1 [COPLANAR] tag match.");
		}
		else if(matches > 1)
		{
			try
			{
				cv::solvePnPGeneric(
					ref->_detect.obj_points,
					ref->_detect.img_points,
					this->calibration,
					this->distortion,
					ref->_detect.rvecs,
					ref->_detect.tvecs,
					false,
					(all_coplanar ? cv::SOLVEPNP_IPPE : cv::SOLVEPNP_SQPNP),	// if coplanar, we want all the solutions so we can manually filter the best
					cv::noArray(),
					cv::noArray(),
					ref->_detect.eerrors);

				RCLCPP_INFO(ref->get_logger(),
					"SolvePnP successfully yielded %lu solution(s) using %lu [%s] tag matches.",
					ref->_detect.tvecs.size(),
					matches,
					all_coplanar ? "COPLANAR" : "non-coplanar");
			}
			catch(const std::exception& e)
			{
				RCLCPP_ERROR(ref->get_logger(), "SolvePnP failed with %lu tag matches: %s", matches, e.what());
			}
		}

		const size_t n_solutions = ref->_detect.tvecs.size();
		if(n_solutions > 0 && n_solutions == ref->_detect.tvecs.size())
		{
			const tf2::TimePoint time_point = tf2::TimePoint{
				std::chrono::seconds{cv_img->header.stamp.sec} +
				std::chrono::nanoseconds{cv_img->header.stamp.nanosec} };
			const geometry_msgs::msg::TransformStamped cam2base =
				ref->tfbuffer.lookupTransform(cv_img->header.frame_id, ref->base_frame_id, time_point);		// camera to base link

			for(size_t i = 0; i < n_solutions; i++)
			{
				cv::Vec3d
					&_rvec = ref->_detect.rvecs[i],
					&_tvec = ref->_detect.tvecs[i];

				cv::drawFrameAxes(this->last_frame, this->calibration, this->distortion, _rvec, _tvec, 0.5f, 5);

				cv::Quatd r = cv::Quatd::createFromRvec(_rvec);
				Eigen::Translation3d& t = reinterpret_cast<Eigen::Translation3d&>(_tvec);

				Eigen::Quaterniond q{ r.w, r.x, r.y, r.z };
				Eigen::Isometry3d _w2cam = (t * q).inverse();	// world to camera
				Eigen::Quaterniond qi;
				Eigen::Vector3d vi;
				qi = _w2cam.rotation();
				vi = _w2cam.translation();

				geometry_msgs::msg::TransformStamped cam2w;		// camera to tags origin
				cam2w.header = cv_img->header;
				cam2w.child_frame_id = std::format("tags_odom_{}", i);
				cam2w.transform.translation = reinterpret_cast<geometry_msgs::msg::Vector3&>(t);
				cam2w.transform.rotation = reinterpret_cast<geometry_msgs::msg::Quaternion&>(q);

				ref->tfbroadcaster.sendTransform(cam2w);

				if(i == 0)
				{
					geometry_msgs::msg::TransformStamped w2cam;	// tags origin to camera
					w2cam.transform.translation = reinterpret_cast<geometry_msgs::msg::Vector3&>(vi);
					w2cam.transform.rotation = reinterpret_cast<geometry_msgs::msg::Quaternion&>(qi);
					w2cam.header.stamp = cv_img->header.stamp;
					w2cam.header.frame_id = ref->tags_frame_id;
					w2cam.child_frame_id = cv_img->header.frame_id;

					geometry_msgs::msg::TransformStamped w2base;	// full transform -- world to base
					tf2::doTransform(cam2base, w2base, w2cam);

					geometry_msgs::msg::PoseWithCovarianceStamped p;
					p.pose.pose.orientation = w2base.transform.rotation;
					p.pose.pose.position = reinterpret_cast<geometry_msgs::msg::Point&>(w2base.transform.translation);
					p.header = w2base.header;

					ref->pose_pub->publish(p);
				}
			}
		}
	}
	else
	{
		// RCLCPP_INFO(ref->get_logger(), "No tags detected in source frame.");
	}

	auto t = std::chrono::system_clock::now();
	if(t - ref->last_publish > ref->target_pub_duration)
	{
		ref->last_publish = t;
		ref->publish_debug_frame();
	}
}

void ArucoServer::ImageSource::info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
{
	if(
		this->valid_calib_data ||
		info->k.size() != 9 ||
		info->d.size() < 5
	) return;

	this->calibration = cv::Mat(info->k, true);
	this->calibration = this->calibration.reshape(0, 3);
	this->distortion = cv::Mat(info->d, true);
	this->distortion = this->distortion.reshape(0, 1);

	this->valid_calib_data = true;

	RCLCPP_INFO(ref->get_logger(), "calib: [%dx%d], distort: [%dx%d] -- %s",
		this->calibration.rows,
		this->calibration.cols,
		this->distortion.rows,
		this->distortion.cols,
		(std::stringstream{} << this->distortion).str().c_str() );

	// try
	// {
	// 	this->undistorted_calib = cv::getOptimalNewCameraMatrix(
	// 		this->calibration,
	// 		this->distortion,
	// 		cv::Size(info->width, info->height),
	// 		1.0,
	// 		cv::Size(info->width, info->height));
	// }
	// catch(const std::exception& e)
	// {
	// 	RCLCPP_ERROR(ref->get_logger(), "Error getting undistorted camera matrix: %s", e.what());
	// }
}


ArucoServer::TagDescription::Ptr ArucoServer::TagDescription::fromRaw(const std::vector<double>& pts)
{
	if(pts.size() < 12) return nullptr;
	Ptr _desc = std::make_shared<TagDescription>();

	const cv::Point3d
		*_0 = reinterpret_cast<const cv::Point3d*>(pts.data() + 0),
		*_1 = reinterpret_cast<const cv::Point3d*>(pts.data() + 3),
		*_2 = reinterpret_cast<const cv::Point3d*>(pts.data() + 6),
		*_3 = reinterpret_cast<const cv::Point3d*>(pts.data() + 9);

	cv::Matx33d rmat;	// rotation matrix from orthogonal axes
	cv::Vec3d			// fill in each row
		*a = reinterpret_cast<cv::Vec3d*>(rmat.val + 0),
		*b = reinterpret_cast<cv::Vec3d*>(rmat.val + 3),
		*c = reinterpret_cast<cv::Vec3d*>(rmat.val + 6);

	*a = *_1 - *_0;	// x-axis

	const double
		len = cv::norm(*a),
		half_len = len / 2.;

	*b = *_1 - *_2;	// y-axis
	*c = ( *a /= len ).cross( *b /= len );	// z-axis can be dervied from x and y

	_desc->world_corners = {
		static_cast<cv::Point3f>(*_0),
		static_cast<cv::Point3f>(*_1),
		static_cast<cv::Point3f>(*_2),
		static_cast<cv::Point3f>(*_3) 
	};
	_desc->rel_corners = {
		cv::Point3f{ -half_len, +half_len, 0.f },
		cv::Point3f{ +half_len, +half_len, 0.f },
		cv::Point3f{ +half_len, -half_len, 0.f },
		cv::Point3f{ -half_len, -half_len, 0.f }
	};
	reinterpret_cast<cv::Point3d&>(_desc->translation) = (*_0 + *_2) / 2.;
	reinterpret_cast<cv::Quatd&>(_desc->rotation) = cv::Quatd::createFromRotMat(rmat);
	reinterpret_cast<cv::Vec3d&>(_desc->plane) = *c;
	_desc->d = c->dot(reinterpret_cast<cv::Vec3d&>(_desc->translation));
	reinterpret_cast<cv::Vec4d&>(_desc->plane) /= cv::norm(reinterpret_cast<cv::Vec4d&>(_desc->plane));		// Eigen cast and normalize() causes crash :|

	return _desc;
}


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoServer>());
	rclcpp::shutdown();

	return 0;
}
