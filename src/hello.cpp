#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuListener : public rclcpp::Node
{
public:
    ImuListener() : Node("imu_listener")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autodrive/f1tenth_1/imu",
            10,
            std::bind(&ImuListener::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Store the current IMU message
        current_imu_message_ = *msg;
        RCLCPP_INFO(this->get_logger(), "IMU data received and stored.");
        RCLCPP_INFO(this->get_logger(), "Linear acceleration: x=%f, y=%f, z=%f",
                    current_imu_message_.linear_acceleration.x,
                    current_imu_message_.linear_acceleration.y,
                    current_imu_message_.linear_acceleration.z);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    sensor_msgs::msg::Imu current_imu_message_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuListener>());
    rclcpp::shutdown();
    return 0;
}
