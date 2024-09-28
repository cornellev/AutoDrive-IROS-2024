#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SimpleDriver : public rclcpp::Node
{
public:
    SimpleDriver() : Node("simple_driver")
    {
        steering_pub_ = create_publisher<std_msgs::msg::Float32>("/control_target/steer_angle", 1);
        // velocity_pub_ = create_publisher<std_msgs::msg::Float32>("/control_target/velocity", 10);
        velocity_pub_ = create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 1);

        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/autodrive/f1tenth_1/lidar", 1, 
            std::bind(&SimpleDriver::scanResponse, this, std::placeholders::_1)
        );
        filtered_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 1, 
            std::bind(&SimpleDriver::saveCurrentOdom, this, std::placeholders::_1)
        );
    }

private:
    void saveCurrentOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_msg_ = *msg;
    }

    void scanResponse(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        lidar_received = true;

        lidar_msg_ = *msg;

        float min_range = -1.5;

        float scan_angle_min = msg->angle_min;
        float scan_angle_max = msg->angle_max;
        float angle_increment = msg->angle_increment;

        int center_index = msg->ranges.size() / 2;

        int start_index = (min_range - scan_angle_min) / angle_increment;
        int end_index = (scan_angle_max + min_range - scan_angle_min) / angle_increment;

        int farthest_angle = start_index;
        float farthest_range = 0.0;

        // Average distance overall
        float avg_distance = 0.0;
        for (int i = start_index; i < end_index; i++) {

            if (msg->ranges[i] > msg->range_max) {
                avg_distance += msg->range_max;
            } else {
                avg_distance += msg->ranges[i];
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "avg=%f", avg_distance);
        avg_distance /= ((float) (end_index - start_index));

        RCLCPP_DEBUG(this->get_logger(), "total=%f", (float) (end_index - start_index));

        RCLCPP_DEBUG(this->get_logger(), "avg_distance=%f", avg_distance);

        // Find point in scan which has the farthest distance
        for (int i = start_index; i < end_index; i++) {
            if (
                (msg->ranges[i] > farthest_range)
            )    
            {
                // Try to keep the angle in the center
                // if (msg->ranges[i] - farthest_range > .3 || abs(i - center_index) < abs(farthest_angle - center_index))
                {
                    farthest_angle = i;
                    farthest_range = msg->ranges[i];
                }
            }
        }

        // Calculate angle and range of farthest point
        float real_angle = scan_angle_min + farthest_angle * angle_increment;
        float s = farthest_range;

        // If s is more than the lidar's farthest distance, set it to the farthest distance
        if (s > msg->range_max)
        {
            s = msg->range_max;
        }

        // Calculate steering angle based on ackermann steering model
        // float steering_angle = atan(2 * wheel_base * sin(real_angle) / s);
        // float steering_angle = real_angle;

        // Use distance left and distance right to adjust steering angle from being too close to walls
        
        // float dist_left = lidar_msg_.ranges[(M_PI / 2.0 - scan_angle_min) / angle_increment];
        // float dist_front_left = lidar_msg_.ranges[(M_PI / 3.0 - scan_angle_min) / angle_increment];

        // float dist_right = lidar_msg_.ranges[(-M_PI / 2.0 - scan_angle_min) / angle_increment];
        // float dist_front_right = lidar_msg_.ranges[(-M_PI / 3.0 - scan_angle_min) / angle_increment];

        float dist_left[4] = {
            lidar_msg_.ranges[(M_PI / 2.0 - scan_angle_min) / angle_increment], // 90
            lidar_msg_.ranges[(M_PI / 3.0 - scan_angle_min) / angle_increment], // 60
            lidar_msg_.ranges[(M_PI / 4.0 - scan_angle_min) / angle_increment], // 45
            lidar_msg_.ranges[(M_PI / 6.0 - scan_angle_min) / angle_increment], // 30
        };

        float dist_right[4] = {
            lidar_msg_.ranges[(-M_PI / 2.0 - scan_angle_min) / angle_increment], // 90
            lidar_msg_.ranges[(-M_PI / 3.0 - scan_angle_min) / angle_increment], // 60
            lidar_msg_.ranges[(-M_PI / 4.0 - scan_angle_min) / angle_increment], // 45
            lidar_msg_.ranges[(-M_PI / 6.0 - scan_angle_min) / angle_increment], // 30
        };

        float dist_front = lidar_msg_.ranges[-scan_angle_min / angle_increment];

        float min_dist_left = *std::min_element(dist_left, dist_left + 4);
        float min_dist_right = *std::min_element(dist_right, dist_right + 4);

        // float new_steering_angle = steering_angle - .05/(pow(dist_left, 2.0)) + .05/(pow(dist_right, 2.9));

        float vel = .01;

        // if (dist_left < .7) {
        //     vel = .005;
        //     steering_angle -= (M_PI / 8.0) * (.7 - dist_left);
        // }
    
        // if (dist_right < .7) {
        //     vel = .005;
        //     steering_angle += (M_PI / 8.0) * (.7 - dist_right);
        // }

        // float new_steering_angle = steering_angle;

        // Publish steering angle and velocity
        auto steering_msg = std_msgs::msg::Float32();

        steering_msg.data = (M_PI / 8.5) * (dist_left[0] - dist_right[0]);
        steering_msg.data += (M_PI / 9.0) * (dist_left[1] - dist_right[1]);
        steering_msg.data += (M_PI / 9.3) * (dist_left[2] - dist_right[2]);
        steering_msg.data += (M_PI / 10.3) * (dist_left[3] - dist_right[3]);

        // steering_msg.data -= atan2(.1, dist_left[3] - dist_right[3]);
        // steering_msg.data -= atan2(.1, dist_left[2] - dist_right[2]);
        // steering_msg.data -= atan2(.1, dist_left[1] - dist_right[1]);
        // steering_msg.data -= atan2(.1, dist_left[0] - dist_right[0]);
        
        // steering_msg.data = new_steering_angle;
        steering_pub_->publish(steering_msg);

        if (dist_front < 1.0) {
            vel = .005;
        }

        // auto velocity_msg = std_msgs::msg::Float32();
        // velocity_msg.data = .2;
        // velocity_pub_->publish(velocity_msg);

        auto velocity_msg = std_msgs::msg::Float32();
        velocity_msg.data = vel;
        velocity_pub_->publish(velocity_msg);

        // RCLCPP_DEBUG(this->get_logger(), "steering=%f", new_steering_angle);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_sub_;

    nav_msgs::msg::Odometry odom_msg_;

    sensor_msgs::msg::LaserScan lidar_msg_;

    bool lidar_received = false;
    float wheel_base = .324; // m

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}