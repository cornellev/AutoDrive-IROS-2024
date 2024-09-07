#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "message_filters/subscriber.h"
#include "control_toolbox/pid.hpp"

class SpeedController : public rclcpp::Node
{
public:
    SpeedController() : Node("speed_controller")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered",
            10,
            std::bind(&SpeedController::topic_callback, this, std::placeholders::_1));
        
        target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/target_speed",
            10,

        );

        publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/throttle",
            10
        );

        pid_ = control_toolbox::Pid(1, 0, 1);
    }

private:
    void topic_callback(const nav_msgs::msg::Odometry msg)
    {

    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    control_toolbox::Pid pid_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedController>());
    rclcpp::shutdown();
    return 0;
}

