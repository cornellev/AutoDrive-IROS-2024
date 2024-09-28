// The following node was created by Myles Patesky, but the commit history could not be preserved when this code was transferred

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class SensorRepublisher : public rclcpp::Node
{
public:
    SensorRepublisher()
    : Node("republish_sensors"),
      tf_buffer_(this->get_clock()),  // Initialize tf2 buffer with the node's clock
      tf_listener_(tf_buffer_)
    {
        RCLCPP_DEBUG(this->get_logger(), "Initializing Sensor Republisher");

        // Create subscriber and publisher
        subscription_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/autodrive/f1tenth_1/lidar", 1, std::bind(&SensorRepublisher::scan_callback, this, std::placeholders::_1));

        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autodrive/f1tenth_1/imu", 1, std::bind(&SensorRepublisher::imu_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 1);

        publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 1);

        // Initialize TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto new_msg = *msg;
        
        new_msg.header.frame_id = "lidar_republished";

        // Publish the modified message
        publisher_->publish(new_msg);

    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto new_msg = *msg;
        
        new_msg.header.frame_id = "imu_republished";

        // Publish the modified message
        publisher_imu_->publish(new_msg);

    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_lidar_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    // tf2 objects
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorRepublisher>());
    rclcpp::shutdown();
    return 0;
}