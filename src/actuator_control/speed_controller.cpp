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
        auto odom_topic = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
        auto velo_topic = declare_parameter<std::string>("target_topic", "/control_target/velocity");
        auto throttle_topic = declare_parameter<std::string>("throttle_topic", "/autodrive/f1tenth_1/throttle_command");

        auto keep_last = rclcpp::QoS(rclcpp::KeepLast(1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, keep_last,
            std::bind(&SpeedController::feedback_callback, this, std::placeholders::_1)
        );

        target_sub_ = create_subscription<std_msgs::msg::Float32>(
            velo_topic, keep_last,
            std::bind(&SpeedController::target_callback, this, std::placeholders::_1)
        );

        publisher_ = create_publisher<std_msgs::msg::Float32>(
            throttle_topic, keep_last
        );

        base_link_frame_ = declare_parameter<std::string>("base_link_frame", "base_link");

        // TODO: dynamic reconfigure
        auto p = declare_parameter<double>("p", 0.1);
        auto i = declare_parameter<double>("i", 0);
        auto d = declare_parameter<double>("d", 0);
        auto max_i = declare_parameter<double>("max_i", 0.0);
        pid_ = control_toolbox::Pid(p, i, d, max_i, -max_i, true);

        RCLCPP_INFO(get_logger(), "Started velocity controller.");
    }

private:
    void feedback_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
    {
        // hard to transform velocities so we're settling for this
        assert(odom->child_frame_id == base_link_frame_);

        rclcpp::Time current(odom->header.stamp);

        if (!last_time_.has_value()) {
            last_time_ = current;
            return;
        }

        rclcpp::Duration dt = current - last_time_.value();
        last_time_ = current;

        double error = target_speed_ - odom->twist.twist.linear.x;
        double command = pid_.computeCommand(error, dt.nanoseconds());

        std_msgs::msg::Float32 msg;
        msg.data = command;
        publisher_->publish(msg);
        RCLCPP_INFO(get_logger(), 
            "Published throttle %f. Target: %f. Actual: %f. Error %f.", 
            msg.data, target_speed_, odom->twist.twist.linear.x, error
        );
    }

    void target_callback(const std_msgs::msg::Float32::ConstSharedPtr target) 
    {
        target_speed_ = target->data;
    }

    float target_speed_ = 0.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

    std::string base_link_frame_;

    control_toolbox::Pid pid_;

    std::optional<rclcpp::Time> last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedController>());
    rclcpp::shutdown();
    return 0;
}
