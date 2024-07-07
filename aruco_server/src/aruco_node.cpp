#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <functional>

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
		Node("aruco_server")
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

		int aruco_dict_id = cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h11;
		util::declare_param(this, "dict_id", aruco_dict_id, aruco_dict_id);

		// TODO: expose detector params

		this->aruco_dict = cv::aruco::getPredefinedDictionary(aruco_dict_id);
		this->aruco_params = cv::aruco::DetectorParameters::create();

		double tag_side_len = 1.0;
		util::declare_param(this, "tag_side_length", tag_side_len, tag_side_len);

		const double corner_off = tag_side_len / 2.0;
		this->tag_relative_corners = {
			cv::Point3f{ -corner_off, +corner_off, 0. },
			cv::Point3f{ +corner_off, +corner_off, 0. },
			cv::Point3f{ +corner_off, -corner_off, 0. },
			cv::Point3f{ -corner_off, -corner_off, 0. }
		};

		this->sources.resize(img_t_sz);
		for(size_t i = 0; i < img_t_sz; i++)
		{
			RCLCPP_DEBUG(this->get_logger(), "Source %ld using image topic [%s] and info topic [%s]", i, img_topics[i].c_str(), info_topics[i].c_str());

			this->sources[i].image_sub =
				this->create_subscription<sensor_msgs::msg::Image>( img_topics[i], 10,
					std::bind( &ArucoServer::img_callback, this, std::placeholders::_1, std::ref(this->sources[i]) ) );
			this->sources[i].info_sub = 
				this->create_subscription<sensor_msgs::msg::CameraInfo>( info_topics[i], 10,
					std::bind( &ArucoServer::info_callback, this, std::placeholders::_1, std::ref(this->sources[i]) ) );
		}
	}

private:
	void img_callback(const sensor_msgs::msg::Image::SharedPtr img, ImageSource& source)
	{
		cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvCopy(*img, "mono8");

		_detect.tag_corners.clear();
		_detect.tag_ids.clear();

		cv::Mat undistorted;
		cv::undistort(
			cv_img->image,
			undistorted,
			source.calibration,
			source.distortion,
			source.undistorted_calib);

		cv::aruco::detectMarkers(
			undistorted,
			aruco_dict,
			_detect.tag_corners,
			_detect.tag_ids,
			aruco_params);

		if(const size_t n_detected = _detect.tag_ids.size(); (n_detected > 0 && n_detected == _detect.tag_corners.size()))	// valid detection(s)
		{
			_detect.eerrors.clear();

			if(n_detected == 1)
			{
				cv::solvePnPGeneric(
					this->tag_relative_corners,
					_detect.tag_corners[0],
					source.undistorted_calib,
					cv::noArray(),
					_detect.rvecs,
					_detect.tvecs,
					false,
					cv::SOLVEPNP_IPPE_SQUARE,
					cv::noArray(),
					cv::noArray(),
					_detect.eerrors);

				// transform and publish estimated base_link pose
			}
			else
			{
				
			}
		}

		
	}

	void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr info, ImageSource& source)
	{
		if(source.has_recieved_info) return;

		source.calibration = cv::Mat(info->k, true);
		if(info->d.size() == 5)
			source.distortion = cv::Mat(info->d, true);

		source.undistorted_calib = cv::getOptimalNewCameraMatrix(
			source.calibration,
			source.distortion,
			cv::Size(info->width, info->height),
			1);

		source.has_recieved_info = true;
	}

	struct ImageSource
	{
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
		cv::Mat1d
			calibration = cv::Mat1f::zeros(3, 3),
			undistorted_calib = cv::Mat1f::zeros(3, 3),
			distortion = cv::Mat1f::zeros(1, 5);

		bool has_recieved_info = false;
	};

	std::vector<ImageSource> sources;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;

	cv::Ptr<cv::aruco::Dictionary> aruco_dict;
	cv::Ptr<cv::aruco::DetectorParameters> aruco_params;
	std::array<cv::Point3d, 4> tag_relative_corners;

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


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoServer>());
	rclcpp::shutdown();

	return 0;
}
