#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class OdometryFusionNode : public rclcpp::Node
{
public:
    OdometryFusionNode() : Node("odometry_fusion")
    {
        // Subscriptions
        wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wheel_odom", 10,
            std::bind(&OdometryFusionNode::wheelOdomCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&OdometryFusionNode::imuCallback, this, std::placeholders::_1));
        
        // Publisher
        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);
        
        // Transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update x and y position from wheel odometry
        fused_odom_.pose.pose.position.x = msg->pose.pose.position.x;
        fused_odom_.pose.pose.position.y = msg->pose.pose.position.y;
        
        // Preserve the velocity from the wheel odometry
        fused_odom_.twist = msg->twist;
        
        // Update the time stamp
        fused_odom_.header.stamp = this->now();
        fused_odom_.header.frame_id = "odom";
        fused_odom_.child_frame_id = "base_link";
        
        publishFusedOdometry();
        broadcastTransform();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Convert quaternion to roll, pitch, and yaw
        tf2::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // Add 90 degrees to the yaw
        double yaw_offset = M_PI / 2.0;
        // yaw = yaw + yaw_offset;

        // Normalize the yaw to keep it within [-π, π]
        // yaw = std::atan2(std::sin(yaw), std::cos(yaw));

        // Update fused orientation
        tf2::Quaternion fused_quat;
        fused_quat.setRPY(0, 0, yaw); // Only yaw is used
        fused_quat.normalize();

        fused_odom_.pose.pose.orientation.x = fused_quat.x();
        fused_odom_.pose.pose.orientation.y = fused_quat.y();
        fused_odom_.pose.pose.orientation.z = fused_quat.z();
        fused_odom_.pose.pose.orientation.w = fused_quat.w();
    }

    void publishFusedOdometry()
    {
        fused_odom_pub_->publish(fused_odom_);
    }

    void broadcastTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        
        transform.header.stamp = fused_odom_.header.stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        
        transform.transform.translation.x = fused_odom_.pose.pose.position.x;
        transform.transform.translation.y = fused_odom_.pose.pose.position.y;
        transform.transform.translation.z = fused_odom_.pose.pose.position.z;
        transform.transform.rotation = fused_odom_.pose.pose.orientation;
        
        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Odometry fused_odom_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryFusionNode>());
    rclcpp::shutdown();
    return 0;
}
