#include "tsq.hpp"
#include "util.hpp"
#include "geometry.hpp"

#include <thread>
#include <atomic>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace util
{
    template<typename ros_T, typename primitive_T>
    inline ros_T to_ros_val(primitive_T v)
    {
        static_assert(std::is_same<typename ros_T::_data_type, primitive_T>::value);

        return ros_T{}.set__data(v);
    }

    inline std_msgs::msg::Float64 to_ros_val(double v)
    {
        return std_msgs::msg::Float64{}.set__data(v);
    }
};

class AccAnalyzer : public rclcpp::Node
{
public:
    AccAnalyzer() :
        rclcpp::Node{ "accuracy_analyzer" },
        tf_buffer{ std::make_shared<rclcpp::Clock>(rclcpp::Clock{ RCL_ROS_TIME }) },
        tf_listener{ tf_buffer }
    {
        util::declare_param(this, "origin_frame_id", this->origin_frame, "map");
        util::declare_param(this, "active_frame_id", this->active_frame, "base_link");
        util::declare_param(this, "validation_frame_id", this->validation_frame, "gz_base_link");
        util::declare_param(this, "std_sample_window_s", this->std_sample_window_s, 0.25);

        this->translational_error_vec_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/accuracy_analyzer/translational_error_vec", rclcpp::BestAvailableQoS{});
        this->translational_std_vec_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/accuracy_analyzer/translational_std_vec", rclcpp::BestAvailableQoS{});
        this->rotational_error_vec_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/accuracy_analyzer/rotational_error_vec", rclcpp::BestAvailableQoS{});
        this->rotational_std_vec_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/accuracy_analyzer/rotational_std_vec", rclcpp::BestAvailableQoS{});
        this->translational_error_pub = this->create_publisher<std_msgs::msg::Float64>("/accuracy_analyzer/translational_error", rclcpp::BestAvailableQoS{});
        this->translational_std_pub = this->create_publisher<std_msgs::msg::Float64>("/accuracy_analyzer/translational_std", rclcpp::BestAvailableQoS{});
        this->rotational_error_pub = this->create_publisher<std_msgs::msg::Float64>("/accuracy_analyzer/rotational_error", rclcpp::BestAvailableQoS{});
        this->rotational_std_pub = this->create_publisher<std_msgs::msg::Float64>("/accuracy_analyzer/rotational_std", rclcpp::BestAvailableQoS{});

        if(!this->active_frame.empty())
        {
            this->is_running = true;
            this->run_thread = std::thread{ &AccAnalyzer::run_analyzer_loop, this };
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[ACCURACY ANALYZER]: No active frame id provided (nothing to analyze)!");
        }
    }

    ~AccAnalyzer()
    {
        this->is_running = false;
        if(this->run_thread.joinable())
        {
            this->run_thread.join();
        }
    }

protected:
    void run_analyzer_loop()
    {
        using namespace util::geom::cvt::ops;
        using Vec6d = Eigen::Vector<double, 6>;

        while(this->is_running)
        {
            rclcpp::Time _now = this->get_clock()->now();
            bool failed = false;

            if(!this->validation_frame.empty())
            {
                // relative accuracy calcs
                try
                {
                    auto tf = this->tf_buffer.lookupTransform(
                        this->active_frame,
                        this->validation_frame,
                        _now,
                        std::chrono::seconds(1) );

                    Eigen::Vector3d _tx;
                    Eigen::Quaterniond _rx;
                    _tx << tf.transform.translation;
                    _rx << tf.transform.rotation;

                    geometry_msgs::msg::Vector3Stamped _vec;
                    _vec.header = tf.header;
                    _vec.vector = tf.transform.translation;
                    this->translational_error_vec_pub->publish(_vec);
                    _vec.vector << _rx.toRotationMatrix().eulerAngles(0, 1, 2);
                    this->rotational_error_vec_pub->publish(_vec);

                    this->translational_error_pub->publish(util::to_ros_val(_tx.norm()));
                    this->rotational_error_pub->publish(util::to_ros_val(_rx.angularDistance(Eigen::Quaterniond::Identity())));
                }
                catch(const std::exception& e)
                {
                    failed = true;
                    RCLCPP_INFO(this->get_logger(), "[ACCURACY ANALYZER]: Tf lookup failed for %s <--> %s transform.\n\twhat(): %s", this->validation_frame.c_str(), this->active_frame.c_str(), e.what());
                }
            }

            if(!this->origin_frame.empty())
            {
                // standard deviation calcs
                try
                {
                    auto tf = this->tf_buffer.lookupTransform(
                        this->active_frame,
                        this->origin_frame,
                        _now,
                        std::chrono::seconds(1) );

                    Vec6d _sample, stdev = Vec6d::Zero();
                    Eigen::Vector3d _tx;
                    Eigen::Quaterniond _rx;
                    _sample.block<1, 3>(0, 0) = (_tx << tf.transform.translation);
                    _sample.block<1, 3>(0, 3) = (_rx << tf.transform.rotation).toRotationMatrix().eulerAngles(0, 1, 2);

                    const double now_f = util::toFloatSeconds(_now);
                    const size_t idx = util::tsq::binarySearchIdx(this->sample_queue, now_f);
                    this->sample_queue.insert( this->sample_queue.begin() + idx, { now_f, _sample } );
                    util::tsq::trimToStamp(this->sample_queue, now_f - this->std_sample_window_s);

                    const size_t n_samples = this->sample_queue.size();
                    if(n_samples > 1)
                    {
                        Vec6d _temp, mean = Vec6d::Zero();
                        for(const auto& s : this->sample_queue)
                        {
                            mean += s.second;
                        }
                        mean /= n_samples;
                        for(const auto& s : this->sample_queue)
                        {
                            _temp = s.second - mean;
                            stdev += _temp.cwiseProduct(_temp);
                        }
                        stdev /= (n_samples - 1);
                        stdev = stdev.cwiseSqrt();
                    }

                    geometry_msgs::msg::Vector3Stamped _vec;
                    _vec.header = tf.header;
                    _tx = stdev.block<1, 3>(0, 0);
                    _vec.vector << _tx;
                    this->translational_std_vec_pub->publish(_vec);
                    this->translational_std_pub->publish(util::to_ros_val(_tx.norm()));
                    _tx = stdev.block<1, 3>(0, 3);
                    _vec.vector << _tx;
                    this->rotational_std_vec_pub->publish(_vec);
                    this->rotational_std_pub->publish(util::to_ros_val(_tx.norm()));
                }
                catch(const std::exception& e)
                {
                    failed = true;
                    RCLCPP_INFO(this->get_logger(), "[ACCURACY ANALYZER]: Tf lookup failed for %s <--> %s transform.\n\twhat(): %s", this->active_frame.c_str(), this->origin_frame.c_str(), e.what());
                }
            }

            if(failed)
            {
                if(!this->is_running) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));    // TODO: sleep_until relative to loop begin time
            }
        }
    }

private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    std::string
        origin_frame,
        active_frame,
        validation_frame;

    double std_sample_window_s;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        translational_error_vec_pub,
        translational_std_vec_pub,
        rotational_error_vec_pub,
        rotational_std_vec_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        translational_error_pub,
        translational_std_pub,
        rotational_error_pub,
        rotational_std_pub;

    util::tsq::TSQ<Eigen::Vector<double, 6>> sample_queue;

    std::thread run_thread;
    std::atomic_bool is_running;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AccAnalyzer>());
    rclcpp::shutdown();

    return 0;
}
