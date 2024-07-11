#include <string>
#include <sstream>
#include <memory>
#include <chrono>
#include <vector>
#include <array>
#include <functional>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

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
private:
	struct ImageSource;
public:
	ArucoServer() :
		Node{ "aruco_server" },
		last_publish{ std::chrono::system_clock::now() },
		img_tp{ std::shared_ptr<ArucoServer>(this, [](auto*){}) },
		tfbuffer{ std::make_shared<rclcpp::Clock>(RCL_ROS_TIME) },
		tflistener{ tfbuffer }
	{
		std::vector<std::string> img_topics, info_topics;
		util::declare_param<std::vector<std::string>>(this, "img_topics", img_topics, {});
		util::declare_param<std::vector<std::string>>(this, "info_topics", info_topics, {});

		std::string dbg_topic;
		util::declare_param(this, "debug_topic", dbg_topic, "aruco_server/debug/image");
		this->debug_pub = this->img_tp.advertise(dbg_topic, 1);

		int pub_freq;
		util::declare_param(this, "debug_pub_freq", pub_freq, 30.);
		this->target_pub_duration = std::chrono::duration<double>{ 1. / pub_freq };

		const size_t
			img_t_sz = img_topics.size(),
			info_t_sz = info_topics.size();

		if(img_t_sz < 1 || info_t_sz < img_t_sz)
		{
			// error -- probably exit
			RCLCPP_ERROR(this->get_logger(), "No image topics provided or mismatched image/info topics");
			return;
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
			RCLCPP_ERROR(this->get_logger(), "No tag info provided!");
			return;
		}

		for(size_t i = 0; i < n_tags; i++)
		{
			param_buff.clear();
			const int id = static_cast<int>(tag_ids[i]);

			std::stringstream topic;
			topic << "tag" << id;
			util::declare_param(this, topic.str(), param_buff, {});

			if(param_buff.size() < 12)
			{
				RCLCPP_ERROR(this->get_logger(), "Parse error or invalid data for tag [%s]", topic.str().c_str());
				continue;
			}

			auto ptr = this->obj_tag_corners.insert(
				{ id, {
					static_cast<cv::Point3f>( *reinterpret_cast<cv::Point3d*>(param_buff.data() + sizeof(cv::Point3d) * 0) ),
					static_cast<cv::Point3f>( *reinterpret_cast<cv::Point3d*>(param_buff.data() + sizeof(cv::Point3d) * 1) ),
					static_cast<cv::Point3f>( *reinterpret_cast<cv::Point3d*>(param_buff.data() + sizeof(cv::Point3d) * 2) ),
					static_cast<cv::Point3f>( *reinterpret_cast<cv::Point3d*>(param_buff.data() + sizeof(cv::Point3d) * 3) ),
				} });
			if(ptr.second)
			{
				// memcpy(ptr.first->second.data(), param_buff.data(), sizeof(cv::Point3d) * 4);
				RCLCPP_INFO(this->get_logger(), "Successfully added corner points for tag %d.", id);
			}
			else this->obj_tag_corners.erase(id);
		}

		this->sources.resize(img_t_sz);
		for(size_t i = 0; i < img_t_sz; i++)
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

private:
	void publish_debug_frame();

	struct ImageSource
	{
		ArucoServer* ref;

		image_transport::Subscriber image_sub;
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;

		cv::Mat last_frame;
		cv::Mat1d
			calibration = cv::Mat1f::zeros(3, 3),
			undistorted_calib = cv::Mat1f::zeros(3, 3),
			distortion = cv::Mat1f::zeros(1, 5);

		bool valid_calib_data = false;

		void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& imf);
		void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);
	};

	std::vector<ImageSource> sources;
	std::chrono::system_clock::time_point last_publish;
	std::chrono::duration<double> target_pub_duration;

	image_transport::ImageTransport img_tp;
	image_transport::Publisher debug_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;

	tf2_ros::Buffer tfbuffer;
	tf2_ros::TransformListener tflistener;
	std::string tags_frame_id, base_frame_id;
	bool filter_prev_proximity, filter_bbox;
	cv::Point3d bbox_min, bbox_max;

	cv::Ptr<cv::aruco::Dictionary> aruco_dict;
	cv::Ptr<cv::aruco::DetectorParameters> aruco_params;
	std::unordered_map<int, std::array<cv::Point3f, 4>> obj_tag_corners;

	struct
	{
		std::vector<std::vector<cv::Point2f>> tag_corners;
		std::vector<int32_t> tag_ids;
		std::vector<cv::Point2f> img_points;
		std::vector<cv::Point3f> obj_points;
		std::vector<cv::Mat1f> tvecs, rvecs;
		std::vector<float> eerrors;
	} _detect;
	

};


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
		catch(...)
		{

		}
	}
}

void ArucoServer::ImageSource::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img)
{
	// RCLCPP_INFO(ref->get_logger(), "FRAME RECIEVED");

	if(!this->valid_calib_data) return;

	cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvCopy(*img, "mono8");

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

	if(const size_t n_detected = ref->_detect.tag_ids.size(); (n_detected > 0 && n_detected == ref->_detect.tag_corners.size()))	// valid detection(s)
	{
		ref->_detect.rvecs.clear();
		ref->_detect.tvecs.clear();
		ref->_detect.eerrors.clear();

		ref->_detect.obj_points.clear();
		ref->_detect.obj_points.reserve(n_detected);
		ref->_detect.img_points.clear();
		ref->_detect.img_points.reserve(n_detected);

		for(size_t i = 0; i < n_detected; i++)
		{
			auto search = ref->obj_tag_corners.find(ref->_detect.tag_ids[i]);
			if(search != ref->obj_tag_corners.end())
			{
				auto& obj_corners = search->second;
				auto& img_corners = ref->_detect.tag_corners[i];

				ref->_detect.obj_points.insert(ref->_detect.obj_points.end(), obj_corners.begin(), obj_corners.end());
				ref->_detect.img_points.insert(ref->_detect.img_points.end(), img_corners.begin(), img_corners.end());
			}
		}

		// RCLCPP_INFO(ref->get_logger(), "MEGATAG solve params -- obj points: %lu, img points: %lu", ref->_detect.obj_points.size(), ref->_detect.img_points.size());

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
				cv::SOLVEPNP_ITERATIVE,
				cv::noArray(),
				cv::noArray(),
				ref->_detect.eerrors);

			RCLCPP_INFO(ref->get_logger(), "SolvePnP successfully yielded %lu solutions from %lu tag detections.", ref->_detect.tvecs.size(), n_detected);
		}
		catch(const std::exception& e)
		{
			RCLCPP_ERROR(ref->get_logger(), "SolvePnP failed with %d tag detections: %s", n_detected, e.what());
		}

		const size_t n_solutions = ref->_detect.tvecs.size();
		if(n_solutions > 0 && n_solutions == ref->_detect.tvecs.size())
		{
			cv::Mat
				_rvec = ref->_detect.rvecs[0],
				_tvec = ref->_detect.tvecs[0];

			if(n_solutions > 1)
			{
				// filter
			}

			Eigen::Vector3d rv;
			Eigen::Translation3d t;

			if(_rvec.depth() == CV_64F)
				rv = *reinterpret_cast<Eigen::Vector3d*>(_rvec.data);
			else if(_rvec.depth() == CV_32F)
				rv = Eigen::Vector3d{ _rvec.at<float>(0), _rvec.at<float>(1), _rvec.at<float>(2) };
			else {}

			if(_tvec.depth() == CV_64F)
				t = *reinterpret_cast<Eigen::Translation3d*>(_tvec.data);
			else if(_tvec.depth() == CV_32F)
				t = Eigen::Translation3d{ _tvec.at<float>(0), _tvec.at<float>(1), _tvec.at<float>(2) };
			else {}

			Eigen::AngleAxisd r{ rv.norm(), rv.normalized() };
			Eigen::Isometry3d _w2cam = (t * r).inverse();	// world to camera
			Eigen::Quaterniond q;
			Eigen::Vector3d v;
			q = _w2cam.rotation();
			v = _w2cam.translation();

			geometry_msgs::msg::TransformStamped w2cam;
			w2cam.transform.rotation = *reinterpret_cast<geometry_msgs::msg::Quaternion*>(&q);
			w2cam.transform.translation = *reinterpret_cast<geometry_msgs::msg::Vector3*>(&v);
			w2cam.header.stamp = cv_img->header.stamp;
			w2cam.header.frame_id = ref->tags_frame_id;
			w2cam.child_frame_id = cv_img->header.frame_id;

			const tf2::TimePoint time_point = tf2::TimePoint{
				std::chrono::seconds{cv_img->header.stamp.sec} +
				std::chrono::nanoseconds{cv_img->header.stamp.nanosec} };
			const geometry_msgs::msg::TransformStamped cam2base =
				ref->tfbuffer.lookupTransform(ref->base_frame_id, cv_img->header.frame_id, time_point);	// camera to base link

			geometry_msgs::msg::TransformStamped w2base;	// full transform -- world to base
			tf2::doTransform(cam2base, w2base, w2cam);

			geometry_msgs::msg::PoseWithCovarianceStamped p;
			p.pose.pose.orientation = w2base.transform.rotation;
			p.pose.pose.position = *reinterpret_cast<geometry_msgs::msg::Point*>(&w2base.transform.translation);
			p.header = w2base.header;

			ref->pose_pub->publish(p);

			// Eigen::Isometry3f cam2base =
			// 	Eigen::Translation3f{
			// 		tf.transform.translation.x,
			// 		tf.transform.translation.y,
			// 		tf.transform.translation.z } *
			// 	Eigen::Quaternionf{
			// 		tf.transform.rotation.w,
			// 		tf.transform.rotation.x,
			// 		tf.transform.rotation.y,
			// 		tf.transform.rotation.z };

			// Eigen::Isometry3f transform = w2cam * cam2base;
			// Eigen::Quaternionf q;
			// q = transform.rotation();

			// cv::Mat1f R, t;
			// cv::Rodrigues(_rvec, R);
			// R = R.t();
			// t = -R * _tvec;


			// geometry_msgs::msg::Transform t;

		}
	}
	else
	{
		RCLCPP_INFO(ref->get_logger(), "No tags detected in source frame.");
	}

	// cv::Mat debug;
	cv::cvtColor(cv_img->image, this->last_frame, CV_GRAY2BGR);
	cv::aruco::drawDetectedMarkers(this->last_frame, ref->_detect.tag_corners, ref->_detect.tag_ids, cv::Scalar{0, 255, 0});

	auto t = std::chrono::system_clock::now();
	if(t - ref->last_publish > ref->target_pub_duration)
	{
		ref->last_publish = t;
		ref->publish_debug_frame();
	}
}

void ArucoServer::ImageSource::info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
{
	// RCLCPP_INFO(ref->get_logger(), "INFO RECIEVED");

	if(this->valid_calib_data) return;
	if(info->k.size() == 9 && info->d.size() == 5)
	{
		this->calibration = cv::Mat(info->k, true);
		this->calibration = this->calibration.reshape(0, 3);
		this->distortion = cv::Mat(info->d, true);
		this->distortion = this->distortion.reshape(0, 1);
	}

	RCLCPP_INFO(ref->get_logger(), "calib: [%dx%d], distort: [%dx%d]", this->calibration.rows, this->calibration.cols, this->distortion.rows, this->distortion.cols);

	try
	{
		this->undistorted_calib = cv::getOptimalNewCameraMatrix(
			this->calibration,
			this->distortion,
			cv::Size(info->width, info->height),
			0.5,
			cv::Size(info->width, info->height));

		this->valid_calib_data = true;
	}
	catch(const std::exception& e)
	{
		RCLCPP_ERROR(ref->get_logger(), "Error getting undistorted camera matrix: %s", e.what());
	}
}


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoServer>());
	rclcpp::shutdown();

	return 0;
}
