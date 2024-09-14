#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class TrajectoryFollower : public rclcpp::Node
{
public:
    TrajectoryFollower() : Node("trajectory_follower")
    {
        steering_pub_ = create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);
        throttle_pub_ = create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TrajectoryFollower::publishCommands, this));
    }

private:
    void publishCommands()
    {
        // Calculate throttle and steering angle based on trajectory

        // Publish throttle command
        auto throttle_msg = std_msgs::msg::Float32();
        throttle_msg.data = calculateThrottle();
        throttle_pub_->publish(throttle_msg);

        // Publish steering command
        auto steering_msg = std_msgs::msg::Float32();
        steering_msg.data = calculateSteeringAngle();
        steering_pub_->publish(steering_msg);
    }

    float calculateThrottle()
    {
        // Calculate throttle based on trajectory
        // Replace with your own implementation
        return 0.5;
    }

    float calculateSteeringAngle()
    {
        // Calculate steering angle based on trajectory
        // Replace with your own implementation
        return -0.3;
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}