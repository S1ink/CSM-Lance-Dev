#include <string>
#include <sstream>
#include <memory>
#include <chrono>
#include <vector>
#include <array>
#include <functional>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
		img_tp{ std::shared_ptr<ArucoServer>(this, [](auto*){}) },
		tfbuffer{ std::make_shared<rclcpp::Clock>(RCL_ROS_TIME) },
		tflistener{ tfbuffer }
	{
		std::vector<std::string> img_topics, info_topics;
		util::declare_param<std::vector<std::string>>(this, "img_topics", img_topics, {});
		util::declare_param<std::vector<std::string>>(this, "info_topics", info_topics, {});

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
				// this->create_subscription<sensor_msgs::msg::Image>( img_topics[i], 10,
				// 	std::bind( &ArucoServer::ImageSource::img_callback, &this->sources[i], std::placeholders::_1 ) );
					// [](const sensor_msgs::msg::Image::SharedPtr img){} );
			this->sources[i].info_sub =
				this->create_subscription<sensor_msgs::msg::CameraInfo>( info_topics[i], 10,
					std::bind( &ArucoServer::ImageSource::info_callback, &this->sources[i], std::placeholders::_1 ) );
					// [](const sensor_msgs::msg::CameraInfo::SharedPtr info){} );
		}
	}

private:
	struct ImageSource
	{
		ArucoServer* ref;

		// rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
		image_transport::Subscriber image_sub;
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
		cv::Mat1d
			calibration = cv::Mat1f::zeros(3, 3),
			undistorted_calib = cv::Mat1f::zeros(3, 3),
			distortion = cv::Mat1f::zeros(1, 5);

		bool valid_calib_data = false;

		void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& imf);
		void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);
	};

	image_transport::ImageTransport img_tp;
	std::vector<ImageSource> sources;
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


void ArucoServer::ImageSource::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img)
{
	RCLCPP_INFO(ref->get_logger(), "FRAME RECIEVED");

	if(!this->valid_calib_data) return;

	cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvCopy(*img, "mono8");

	ref->_detect.tag_corners.clear();
	ref->_detect.tag_ids.clear();

	try {
		cv::Mat undistorted;
		cv::undistort(
			cv_img->image,
			undistorted,
			this->calibration,
			this->distortion,
			this->undistorted_calib);
	}
	catch(const std::exception& e)
	{
		RCLCPP_ERROR(ref->get_logger(), "Error undistorting image: %s", e.what());
	}

	try {
		cv::aruco::detectMarkers(
			cv_img->image, // undistorted,
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

		if(n_detected == 1)
		{
			auto search = ref->obj_tag_corners.find(ref->_detect.tag_ids[0]);
			if(search != ref->obj_tag_corners.end())
			{
				auto& corners = search->second;

				cv::solvePnPGeneric(
					corners,
					ref->_detect.tag_corners[0],
					this->undistorted_calib,
					cv::noArray(),
					ref->_detect.rvecs,
					ref->_detect.tvecs,
					false,
					cv::SOLVEPNP_IPPE_SQUARE,
					cv::noArray(),
					cv::noArray(),
					ref->_detect.eerrors);
			}
		}
		else
		{
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

			RCLCPP_INFO(ref->get_logger(), "MEGATAG solve params -- obj points: %lu, img points: %lu", ref->_detect.obj_points.size(), ref->_detect.img_points.size());

			try
			{
				cv::solvePnPGeneric(
					ref->_detect.obj_points,
					ref->_detect.img_points,
					this->undistorted_calib,
					cv::noArray(),
					ref->_detect.rvecs,
					ref->_detect.tvecs,
					false,
					cv::SOLVEPNP_ITERATIVE,
					cv::noArray(),
					cv::noArray(),
					ref->_detect.eerrors);
			}
			catch(const std::exception& e)
			{
				RCLCPP_ERROR(ref->get_logger(), "MEGATAG solvePnP failed: %s", e.what());
			}
		}

		const size_t n_solutions = ref->_detect.rvecs.size();
		// if(n_solutions > 0 && n_solutions == ref->_detect.tvecs.size())
		// {
		// 	if(n_solutions > 1)
		// 	{
		// 		// filter
		// 	}

		// }
		RCLCPP_INFO(ref->get_logger(), "SolvePnP successfully yielded %lu solutions from %lu tag detections.", n_solutions, n_detected);
	}
	else
	{
		RCLCPP_INFO(ref->get_logger(), "No tags detected in source frame.");
	}
}

void ArucoServer::ImageSource::info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
{
	RCLCPP_INFO(ref->get_logger(), "INFO RECIEVED");

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
			1,
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
