#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class AckermannOdometry : public rclcpp::Node
{
public:
    AckermannOdometry() : Node("ackermann_odometry")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Ackermann Odometry Node");

        // Parameters
        wheelbase_ = 0.3240;
        wheel_radius_ = 0.0590;
        encoder_ticks_per_rotation_ = 16.0;
        prev_time_ = this->now();

        // Publishers and Subscriptions
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);

        left_encoder_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/autodrive/f1tenth_1/left_encoder", 10, 
            std::bind(&AckermannOdometry::leftEncoderCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, 
            std::bind(&AckermannOdometry::imuCallback, this, std::placeholders::_1));

        throttle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/throttle", 10,
            std::bind(&AckermannOdometry::throttleCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void leftEncoderCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!left_encoder_initialized_) {
            old_left_rotations = msg->position[0] / encoder_ticks_per_rotation_;
            new_left_rotations = old_left_rotations;
            left_encoder_initialized_ = true;
        } else {
            new_left_rotations = msg->position[0] / encoder_ticks_per_rotation_;

            timerCallback();
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Convert quaternion to RPY (roll, pitch, yaw)
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        current_orientation_ = yaw;
    }

    void throttleCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        throttle_ = msg->data;
    }

    double encoderToRotations(double encoder_value) 
    {
        return encoder_value / encoder_ticks_per_rotation_;
    }

    void initializeEncoder(bool &encoder_initialized, double &initial_encoder, double encoder_value)
    {
        if (!encoder_initialized)
        {
            initial_encoder = encoderToRotations(encoder_value);
            encoder_initialized = true;
        }
    }

    void timerCallback()
    {
        auto current_time = this->now();
        double dt = (current_time - prev_time_).seconds();
        if (dt < 1e-6) return;

        if (fabs(new_left_rotations - old_left_rotations) < 1e-6)
        {
            publishOdometry(current_time, 0.0, 0.0); // Publish 0 velocity
            return;
        }

        double delta_left = new_left_rotations - old_left_rotations;
        
        double delta_distance = delta_left * 2.0 * M_PI * wheel_radius_;
        double linear_velocity = delta_distance / dt;

        // Update position using current orientation (from IMU)
        x_ += delta_distance * std::cos(current_orientation_);
        y_ += delta_distance * std::sin(current_orientation_);

        // Angular velocity is derived from IMU orientation changes
        double angular_velocity = (current_orientation_ - old_orientation_) / dt;

        // Publish odometry
        publishOdometry(current_time, linear_velocity, angular_velocity);

        // Broadcast the transform from base_link to odom
        broadcastTransform(current_time);

        old_left_rotations = new_left_rotations;
        old_orientation_ = current_orientation_;

        prev_time_ = current_time;
    }

    void publishOdometry(const rclcpp::Time &current_time, double linear_velocity, double angular_velocity)
    {
        /**
         * Publish odometry message using current pose/twist along with calculated velocities.
         */
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_orientation_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = linear_velocity;
        odom_msg.twist.twist.angular.z = angular_velocity;

        for (int i = 0; i < 36; i++)
        {
            odom_msg.pose.covariance[i] = covariance_[i];
        }

        odom_publisher_->publish(odom_msg);
    }

    void broadcastTransform(const rclcpp::Time &current_time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = current_time;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";

        transformStamped.transform.translation.x = x_;
        transformStamped.transform.translation.y = y_;
        transformStamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_orientation_);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_encoder_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double wheelbase_;
    double wheel_radius_;
    double encoder_ticks_per_rotation_;

    double throttle_ = 0.0;

    double x_ = 0.0;
    double y_ = 0.0;
    double current_orientation_ = 0.0;
    double old_orientation_ = 0.0;

    double old_left_rotations = 0.0;
    double new_left_rotations = 0.0;

    bool left_encoder_initialized_ = false;

    double covariance_[36] = {
        0.01, 0, 0, 0, 0, 0,
        0, 0.01, 0, 0, 0, 0,
        0, 0, 0.01, 0, 0, 0,
        0, 0, 0, 0.01, 0, 0,
        0, 0, 0, 0, 0.01, 0,
        0, 0, 0, 0, 0, 0.01
    };
    rclcpp::Time prev_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannOdometry>());
    rclcpp::shutdown();
    return 0;
}
