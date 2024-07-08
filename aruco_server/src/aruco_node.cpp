#include <string>
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

		const size_t n_tags = param_buff.size();
		if(n_tags < 1)
		{
			RCLCPP_ERROR(this->get_logger(), "No tag info provided!");
			return;
		}

		for(size_t i = 0; i < n_tags; i++)
		{
			param_buff.clear();
			const int id = static_cast<int>(tag_ids[i]);

			util::declare_param(this, "tag" + id, param_buff, {});

			if(param_buff.size() < 12) continue;

			auto ptr = this->obj_tag_corners.insert({id, {}});
			if(ptr.second)
			{
				memcpy(ptr.first->second.data(), param_buff.data(), sizeof(cv::Point3d) * 4);
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
				this->create_subscription<sensor_msgs::msg::Image>( img_topics[i], 10,
					std::bind( &ArucoServer::ImageSource::img_callback, &this->sources[i], std::placeholders::_1 ) );
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

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
		cv::Mat1d
			calibration = cv::Mat1f::zeros(3, 3),
			undistorted_calib = cv::Mat1f::zeros(3, 3),
			distortion = cv::Mat1f::zeros(1, 5);

		bool has_recieved_info = false;

		void img_callback(const sensor_msgs::msg::Image::SharedPtr imf);
		void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr info);
	};

	std::vector<ImageSource> sources;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;

	tf2_ros::Buffer tfbuffer;
	tf2_ros::TransformListener tflistener;
	std::string tags_frame_id, base_frame_id;
	bool filter_prev_proximity, filter_bbox;
	cv::Point3d bbox_min, bbox_max;

	cv::Ptr<cv::aruco::Dictionary> aruco_dict;
	cv::Ptr<cv::aruco::DetectorParameters> aruco_params;
	std::unordered_map<int, std::array<cv::Point3d, 4>> obj_tag_corners;

	struct
	{
		std::vector<std::vector<cv::Point2d>> tag_corners;
		std::vector<int32_t> tag_ids;
		std::vector<cv::Point2d> img_points;
		std::vector<cv::Point3d> obj_points;
		std::vector<cv::Mat1f> tvecs, rvecs;
		std::vector<float> eerrors;
	} _detect;
	

};


void ArucoServer::ImageSource::img_callback(const sensor_msgs::msg::Image::SharedPtr img)
{
	RCLCPP_INFO(ref->get_logger(), "FRAME RECIEVED");

	if(!this->has_recieved_info) return;

	cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvCopy(*img, "mono8");

	ref->_detect.tag_corners.clear();
	ref->_detect.tag_ids.clear();

	// cv::Mat undistorted;
	// cv::undistort(
	// 	cv_img->image,
	// 	undistorted,
	// 	this->calibration,
	// 	this->distortion,
	// 	this->undistorted_calib);

	cv::aruco::detectMarkers(
		cv_img->image, // undistorted,
		ref->aruco_dict,
		ref->_detect.tag_corners,
		ref->_detect.tag_ids,
		ref->aruco_params);

/*
[aruco_node-8] terminate called after throwing an instance of 'cv::Exception'
[aruco_node-8]   what():  OpenCV(4.6.0) ./modules/core/src/matrix_wrap.cpp:1393: error: (-215:Assertion failed) mtype == type0 || (CV_MAT_CN(mtype) == CV_MAT_CN(type0) && ((1 << type0) & fixedDepthMask) != 0) in function 'create'
*/

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
					this->calibration, // this->undistorted_calib,
					this->distortion, // cv::noArray(),
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

			cv::solvePnPGeneric(
				ref->_detect.obj_points,
				ref->_detect.img_points,
				this->calibration, // this->undistorted_calib,
				this->distortion, // cv::noArray(),
				ref->_detect.rvecs,
				ref->_detect.tvecs,
				false,
				cv::SOLVEPNP_SQPNP,
				cv::noArray(),
				cv::noArray(),
				ref->_detect.eerrors);
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

void ArucoServer::ImageSource::info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr info)
{
	RCLCPP_INFO(ref->get_logger(), "INFO RECIEVED");

	if(this->has_recieved_info) return;

	this->calibration = cv::Mat(info->k, true);
	this->calibration.reshape(1, 3);
	if(info->d.size() == 5)
		this->distortion = cv::Mat(info->d, true);

	RCLCPP_INFO(ref->get_logger(), "calib: %d, distort: %d", this->calibration.size().area(), this->distortion.size().area());

	try
	{
		this->undistorted_calib = cv::getOptimalNewCameraMatrix(
			this->calibration,
			this->distortion,
			cv::Size(info->width, info->height),
			1);
	}
	catch(const std::exception& e)
	{
		RCLCPP_ERROR(ref->get_logger(), "Error getting undistorted camera matrix");
	}

	this->has_recieved_info = true;
}


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoServer>());
	rclcpp::shutdown();

	return 0;
}
