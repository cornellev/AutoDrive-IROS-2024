// The following node was created by Myles Patesky, but the commit history could not be preserved when this code was transferred

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class RepublishScanNode : public rclcpp::Node
{
public:
    RepublishScanNode()
    : Node("republish_scan_node"),
      tf_buffer_(this->get_clock()),  // Initialize tf2 buffer with the node's clock
      tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Republish Scan Node");

        // Declare parameters
        this->declare_parameter<std::string>("old_sensor_topic", "/autodrive/f1tenth_1/lidar");
        this->declare_parameter<std::string>("new_sensor_topic", "/scan");
        this->declare_parameter<std::string>("new_frame_id", "lidar_republished");
        this->declare_parameter<std::string>("old_base_link_frame", "f1tenth_1");
        this->declare_parameter<std::string>("old_lidar_frame", "lidar");
        this->declare_parameter<std::string>("new_base_link_frame", "base_link");

        // Get parameters
        old_sensor_topic_ = this->get_parameter("old_sensor_topic").as_string();
        new_sensor_topic_ = this->get_parameter("new_sensor_topic").as_string();
        new_frame_id_ = this->get_parameter("new_frame_id").as_string();
        old_base_link_frame_ = this->get_parameter("old_base_link_frame").as_string();
        old_lidar_frame_ = this->get_parameter("old_lidar_frame").as_string();
        new_base_link_frame_ = this->get_parameter("new_base_link_frame").as_string();

        // Create subscriber and publisher
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            old_sensor_topic_, 10, std::bind(&RepublishScanNode::scan_callback, this, std::placeholders::_1));

        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autodrive/f1tenth_1/imu", 10, std::bind(&RepublishScanNode::imu_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(new_sensor_topic_, 10);

        publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

        // Initialize TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto new_msg = *msg;
        
        new_msg.header.frame_id = new_frame_id_;

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

    std::string old_sensor_topic_;
    std::string new_sensor_topic_;
    std::string new_frame_id_;
    std::string old_base_link_frame_;
    std::string old_lidar_frame_;
    std::string new_base_link_frame_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    // tf2 objects
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RepublishScanNode>());
    rclcpp::shutdown();
    return 0;
}