#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>


class AckermannOdometry : public rclcpp::Node
{
public:
    AckermannOdometry() : Node("ackermann_odometry")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Ackermann Odometry Node");

        // Parameters
        wheelbase_ = 0.3240;
        track_width_ = 0.2360;
        wheel_radius_ = 0.0590;
        encoder_ticks_per_rotation_ = 16.0;
        prev_time_ = this->now();

        // Publish to wheel odometry topic
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 10);

        // Subscribe to encoders and steering angle
        left_encoder_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/autodrive/f1tenth_1/left_encoder",
            10,
            std::bind(&AckermannOdometry::leftEncoderCallback, this, std::placeholders::_1));
        
        right_encoder_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/autodrive/f1tenth_1/right_encoder",
            10,
            std::bind(&AckermannOdometry::rightEncoderCallback, this, std::placeholders::_1));
        
        steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/steering",
            10,
            std::bind(&AckermannOdometry::steeringCallback, this, std::placeholders::_1));
    }

private:
    void leftEncoderCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!left_encoder_initialized_)
        {
            initial_left_encoder_ = encoderToRotations(msg->position[0]);
            left_encoder_initialized_ = true;
        }

        left_encoder_rotations_ = encoderToRotations(msg->position[0]) - initial_left_encoder_;
        computeOdometry();
    }

    void rightEncoderCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!right_encoder_initialized_)
        {
            initial_right_encoder_ = encoderToRotations(msg->position[0]);
            right_encoder_initialized_ = true;
        }

        right_encoder_rotations_ = encoderToRotations(msg->position[0]) - initial_right_encoder_;
        computeOdometry();
    }

    void steeringCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        steering_angle_ = msg->data;
        computeOdometry();
    }

    double encoderToRotations(double encoder_value) 
    {
        return encoder_value / encoder_ticks_per_rotation_;
    }

    void computeOdometry()
    {
        auto current_time = this->now();
        double dt = (current_time - prev_time_).seconds();
        if (dt < 1e-6) return; // Avoid division by zero

        if (fabs(left_encoder_rotations_ - prev_left_encoder_rotations_) < 1e-6 || fabs(right_encoder_rotations_ - prev_right_encoder_rotations_) < 1e-6)
        {
            // Publish with zero velocity but correct position and orientation
            auto odom_msg = nav_msgs::msg::Odometry();

            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            // Set the position
            odom_msg.pose.pose.position.x = x_;
            odom_msg.pose.pose.position.y = y_;
            odom_msg.pose.pose.position.z = 0.0;

            // Set the velocity
            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.angular.z = 0.0;

            // Set the orientation
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, current_orientation_);
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();

            // Publish the message
            odom_publisher_->publish(odom_msg);
            return;
        }

        // Calculate distance traveled by each wheel (based on encoder values)
        double delta_left = (left_encoder_rotations_ - prev_left_encoder_rotations_) * wheel_radius_;
        double delta_right = (right_encoder_rotations_ - prev_right_encoder_rotations_) * wheel_radius_;
        
        // Calculate the average distance the vehicle has traveled
        double delta_distance = (delta_left + delta_right) / 2.0;

        // Calculate the turning radius based on the steering angle (R = L / tan(steering angle))
        double turning_radius = wheelbase_ / std::tan(steering_angle_);

        // Compute the angular velocity (delta_theta) based on the turning radius and distance traveled
        double delta_theta = delta_distance / turning_radius;

        // Update the orientation of the vehicle
        current_orientation_ += delta_theta;

        // Compute change in position using Ackermann model
        double delta_x = delta_distance * std::cos(current_orientation_);
        double delta_y = delta_distance * std::sin(current_orientation_);

        // Update the vehicle's position
        x_ += delta_x;
        y_ += delta_y;

        // Update previous encoder values and time for next iteration
        prev_left_encoder_rotations_ = left_encoder_rotations_;
        prev_right_encoder_rotations_ = right_encoder_rotations_;
        prev_time_ = current_time;

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
        odom_msg.twist.twist.linear.x = delta_distance / dt;
        // odom_msg.twist.twist.angular.z = delta_theta / dt;

        // Set the orientation
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_orientation_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // Publish the message
        odom_publisher_->publish(odom_msg);
        

        // Print the odometry
        RCLCPP_DEBUG(this->get_logger(), "x=%f, y=%f, theta=%f", x_, y_, current_orientation_);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_encoder_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_encoder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;

    double wheelbase_;
    double track_width_;
    double wheel_radius_;
    double encoder_ticks_per_rotation_;
    double prev_left_encoder_rotations_ = 0.0;
    double prev_right_encoder_rotations_ = 0.0;
    double left_encoder_rotations_ = 0.0;
    double right_encoder_rotations_ = 0.0;
    double steering_angle_ = 0.0;
    double x_ = 0.0;
    double y_ = 0.0;
    double current_orientation_ = 0.0;
    double initial_left_encoder_ = 0.0;
    double initial_right_encoder_ = 0.0;
    bool left_encoder_initialized_ = false;
    bool right_encoder_initialized_ = false;
    rclcpp::Time prev_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AckermannOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
