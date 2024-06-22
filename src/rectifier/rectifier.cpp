#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


namespace tf2
{
    template<>
    inline void doTransform(
        const sensor_msgs::msg::Imu& t_in,
        sensor_msgs::msg::Imu& t_out,
        const geometry_msgs::msg::TransformStamped& transform)
    {
        t_out.header.stamp = transform.header.stamp;
        t_out.header.frame_id = transform.header.frame_id;

        doTransform(t_in.orientation, t_out.orientation, transform);
        t_out.orientation_covariance = t_in.orientation_covariance;

        doTransform(t_in.angular_velocity, t_out.angular_velocity, transform);
        t_out.angular_velocity_covariance = t_in.angular_velocity_covariance;

        doTransform(t_in.linear_acceleration, t_out.linear_acceleration, transform);
        t_out.linear_acceleration_covariance = t_in.linear_acceleration_covariance;
    }
}

class Rectifier : public rclcpp::Node
{
public:
    Rectifier() :
        rclcpp::Node{ "rectifier" },
        clock_{ RCL_ROS_TIME },
        tfbuffer_{ std::make_shared<rclcpp::Clock>(clock_) },
        tflistener_{tfbuffer_ }
    {
        this->declare_parameter("base_frame_id", "base_link");
        this->get_parameter("base_frame_id", this->base_frame_id);

        this->imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("rectified_imu", rclcpp::QoS{10});
        this->pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("rectified_pc", rclcpp::QoS{10});

        this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", rclcpp::SensorDataQoS{},
            [this](const typename sensor_msgs::msg::Imu::SharedPtr msg)
            {
                sensor_msgs::msg::Imu rectified;
                const tf2::TimePoint time_point = tf2::TimePoint{ std::chrono::seconds{msg->header.stamp.sec} + std::chrono::nanoseconds{msg->header.stamp.nanosec} };
                const geometry_msgs::msg::TransformStamped tf = this->tfbuffer_.lookupTransform(this->base_frame_id, msg->header.frame_id, time_point);
                tf2::doTransform(*msg, rectified, tf);
                this->imu_pub->publish(rectified);
            }
        );
        this->pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", rclcpp::SensorDataQoS{},
            [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg)
            {
                sensor_msgs::msg::PointCloud2 rectified;
                const tf2::TimePoint time_point = tf2::TimePoint{ std::chrono::seconds{msg->header.stamp.sec} + std::chrono::nanoseconds{msg->header.stamp.nanosec} };
                const geometry_msgs::msg::TransformStamped tf = this->tfbuffer_.lookupTransform(this->base_frame_id, msg->header.frame_id, time_point);
                tf2::doTransform(*msg, rectified, tf);
                this->pc_pub->publish(rectified);
            }
        );
    }

private:
    rclcpp::Clock clock_;
    tf2_ros::Buffer tfbuffer_;
    tf2_ros::TransformListener tflistener_;

    std::string base_frame_id;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Rectifier>());
    rclcpp::shutdown();

    return 0;
}
