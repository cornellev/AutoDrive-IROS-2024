#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <control_toolbox/pid.hpp>
#include <optional>

class SpeedController : public rclcpp::Node
{
public:
    SpeedController() : Node("speed_controller")
    {
        odom_sub_.subscribe(this, "/odometry/filtered");
        target_sub_.subscribe(this, "/target_velocity");

        ts_ = std::make_shared<message_filters::Synchronizer<sync_policy>>(odom_sub_, target_sub_, 10);
        ts_->registerCallback(std::bind(&SpeedController::callback, this, std::placeholders::_1, std::placeholders::_2));

        publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/throttle",
            10
        );

        // TODO: dynamic reconfigure
        pid_ = control_toolbox::Pid(1, 0, 1);
    }

private:
    void callback(const nav_msgs::msg::Odometry::ConstPtr odom, const std_msgs::msg::Float32::ConstPtr target)
    {
        // TODO: use transform to avoid this, also take base link as parameter
        assert(odom->child_frame_id == "base_link");

        if (!last_time.has_value()) return;

        rclcpp::Duration dt = rclcpp::Time(odom->header.stamp) - last_time.value();
        double error = target->data - odom->twist.twist.linear.x;

        double command = pid_.computeCommand(error, dt.nanoseconds());
        publisher_->publish(command);
    }

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, std_msgs::msg::Float32> sync_policy;

    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<std_msgs::msg::Float32> target_sub_;
    std::shared_ptr<message_filters::Synchronizer<sync_policy>> ts_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    control_toolbox::Pid pid_;

    std::optional<rclcpp::Time> last_time;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedController>());
    rclcpp::shutdown();
    return 0;
}

