#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <optional>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <memory>

const double WHEEL_BASE = 0.3240;
const double TRACK_WIDTH = 0.2360;
const double WHEEL_RADIUS = 0.0590;
const double WHEEL_CIRC = 2 * M_PI * WHEEL_RADIUS;
const double ENCODER_TICKS_PER_REV = 32;

class AckermannOdometry : public rclcpp::Node
{
public:
    AckermannOdometry() : Node("ackermann_odometry")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Ackermann odometry node");

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom",
            rclcpp::QoS(rclcpp::KeepLast(10))
        );

        rclcpp::SensorDataQoS sensor_qos;
        left_encoder_sub_.subscribe(this, "/autodrive/f1tenth_1/left_encoder", sensor_qos.get_rmw_qos_profile());
        right_encoder_sub_.subscribe(this, "/autodrive/f1tenth_1/right_encoder", sensor_qos.get_rmw_qos_profile());
        steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/steering", sensor_qos,
            std::bind(&AckermannOdometry::steerCallback, this, std::placeholders::_1));

        sync_ = std::make_shared<message_filters::Synchronizer<sync_policy>>(
            sync_policy(10),
            left_encoder_sub_,
            right_encoder_sub_
        );
        sync_->setAgePenalty(0.1);  // 10 ms

        sync_->registerCallback(
            std::bind(
                &AckermannOdometry::encoderCallback, this,
                std::placeholders::_1, std::placeholders::_2
            )
        );

        RCLCPP_INFO(this->get_logger(), "Started Ackermann odometry");
    }

private:
    void steerCallback(std_msgs::msg::Float32::ConstSharedPtr msg) {
        latest_steering_ = msg->data;
    }

    void encoderCallback(
        const sensor_msgs::msg::JointState::ConstSharedPtr &left_encoder, 
        const sensor_msgs::msg::JointState::ConstSharedPtr &right_encoder)
    {
        rclcpp::Time current_time = left_encoder->header.stamp;

        if (!has_last) {
            has_last = true;
            last_time_ = current_time;
            last_left_ = left_encoder;
            last_right_ = right_encoder;
            return;
        }

        rclcpp::Duration dt = current_time - last_time_;

        double left_encoder_rotations = left_encoder->position[0] / ENCODER_TICKS_PER_REV;
        double right_encoder_rotations = right_encoder->position[0] / ENCODER_TICKS_PER_REV;
        double prev_left_encoder_rotations = last_left_->position[0] / ENCODER_TICKS_PER_REV;
        double prev_right_encoder_rotations = last_right_->position[0] / ENCODER_TICKS_PER_REV;
        
        double steering_angle = latest_steering_;

        // Calculate distance traveled by each wheel (based on encoder values)
        double delta_left = (left_encoder_rotations - prev_left_encoder_rotations) * WHEEL_CIRC;
        double delta_right = (right_encoder_rotations - prev_right_encoder_rotations) * WHEEL_CIRC;
        
        // Calculate the average distance the vehicle has traveled
        double delta_distance = (delta_left + delta_right) / 2.0;

        // Calculate the turning radius based on the steering angle (R = L / tan(steering angle))
        // I think this actually does work when steering_angle = 0 due to float magic
        double turning_radius = WHEEL_BASE / std::tan(steering_angle);

        // Compute the angular velocity (delta_theta) based on the turning radius and distance traveled
        double delta_theta = delta_distance / turning_radius;

        // Update the orientation of the vehicle
        theta_ += delta_theta;

        // Compute change in position using Ackermann model
        double delta_x = delta_distance * std::cos(theta_);
        double delta_y = delta_distance * std::sin(theta_);

        // Update the vehicle's position
        x_ += delta_x;
        y_ += delta_y;

        // Calculate velocity
        double forward_vel = delta_distance / dt.seconds();

        // Update previous encoder values and time for next iteration
        last_time_ = current_time;
        last_left_ = left_encoder;
        last_right_ = right_encoder;

        // Publish the odometry message
        auto odom_msg = nav_msgs::msg::Odometry();

        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Set the position
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // Set the velocity
        odom_msg.twist.twist.linear.x = forward_vel;

        // Set the orientation
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // Publish the message
        odom_publisher_->publish(odom_msg);
        
        // Print the odometry
        RCLCPP_DEBUG(this->get_logger(), "x=%f, y=%f, theta=%f", x_, y_, theta_);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    message_filters::Subscriber<sensor_msgs::msg::JointState> left_encoder_sub_;
    message_filters::Subscriber<sensor_msgs::msg::JointState> right_encoder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;  // have to do this as a normal subscription since no header

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::JointState, 
        sensor_msgs::msg::JointState> sync_policy;
    std::shared_ptr<message_filters::Synchronizer<sync_policy>> sync_;

    bool has_last = false;
    rclcpp::Time last_time_;
    sensor_msgs::msg::JointState::ConstSharedPtr last_left_;
    sensor_msgs::msg::JointState::ConstSharedPtr last_right_;

    double latest_steering_ = 0.0;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AckermannOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
