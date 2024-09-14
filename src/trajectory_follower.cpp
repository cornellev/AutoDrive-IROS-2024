#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class TrajectoryFollower : public rclcpp::Node
{
public:
    TrajectoryFollower() : Node("trajectory_follower")
    {
        steering_pub_ = create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);
        throttle_pub_ = create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/autodrive/f1tenth_1/lidar", 10, 
            std::bind(&TrajectoryFollower::saveCurrentScan, this, std::placeholders::_1)
        );
        filtered_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, 
            std::bind(&TrajectoryFollower::saveCurrentOdom, this, std::placeholders::_1)
        );

        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TrajectoryFollower::publishCommands, this));
    }

private:
    void publishCommands()
    {
        // Calculate throttle and steering angle based on trajectory

        // Publish throttle command
        auto throttle_msg = std_msgs::msg::Float32();
        throttle_msg.data = calculateThrottle(.6);
        throttle_pub_->publish(throttle_msg);

        // Publish steering command
        auto steering_msg = std_msgs::msg::Float32();
        steering_msg.data = calculateSteeringAngle();
        steering_pub_->publish(steering_msg);
    }

    void saveCurrentScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Print the entire scan
        // for (long unsigned int i = 0; i < msg->ranges.size(); i++)
        // {
        //     // RCLCPP_INFO(this->get_logger(), "Range[%d]: %f", i, msg->ranges[i]);
        // }

        // Find location in laserscan with maximum distance that is larger than my car width

        // float minimum_angle = .340 // 20 degrees

        float min_range = -1.22;
        // float max_range = .52; // 30 degrees

        float scan_angle_min = msg->angle_min;
        float scan_angle_max = msg->angle_max;
        float angle_increment = msg->angle_increment;

        int center_index = msg->ranges.size() / 2;

        // Print angle increment, scan angle min, scan angle max
        // RCLCPP_INFO(this->get_logger(), "Angle increment: %f", angle_increment);
        // RCLCPP_INFO(this->get_logger(), "Scan angle min: %f", scan_angle_min);
        // RCLCPP_INFO(this->get_logger(), "Scan angle max: %f", scan_angle_max);

        int start_index = (min_range - scan_angle_min) / angle_increment;

        // Print start index here
        // RCLCPP_INFO(this->get_logger(), "Start index Again: %d", start_index);
        int end_index = (scan_angle_max + min_range - scan_angle_min) / angle_increment;

        // Print end index here
        // RCLCPP_INFO(this->get_logger(), "End index: %d", end_index);


        int farthest_angle = start_index;
        float farthest_range = 0.0;

        for (int i = start_index; i < end_index; i++) {
            // RCLCPP_INFO(this->get_logger(), "Range[%d]: %f", i, msg->ranges[i]);


            if (
                (msg->ranges[i] > farthest_range) ||
                (
                    (
                        fabs(msg->ranges[i] - farthest_range) < .1
                    ) &&
                    (fabs(i - center_index) < fabs(farthest_angle - center_index))
                )
            )    
            {
                farthest_angle = i;
                farthest_range = msg->ranges[i];
            }
        }

        float real_angle = scan_angle_min + farthest_angle * angle_increment;
        float s = farthest_range;

        // If s is more than the lidar's farthest distance, set it to the farthest distance
        if (s > msg->range_max)
        {
            s = msg->range_max;
        }

        float steering_angle = atan(3 * wheel_base * sin(real_angle) / s);

        RCLCPP_INFO(this->get_logger(), "FARTHEST INDEX: %d, %f", farthest_angle, farthest_range);
        RCLCPP_INFO(this->get_logger(), "STEERING ANGLE: %f", steering_angle);




        // Print angle min and angle max
        // RCLCPP_INFO(this->get_logger(), "Angle min: %f", msg->angle_min);
        // RCLCPP_INFO(this->get_logger(), "Angle max: %f", msg->angle_max);

        // Print the range at the center of the scan
        // RCLCPP_INFO(this->get_logger(), "Center range: %f", msg->ranges[msg->ranges.size() / 2]);
    }

    void saveCurrentOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_msg_ = *msg;
    }

    float calculateThrottle(float desired_velocity)
    {
        float velocity = odom_msg_.twist.twist.linear.x;
        float throttle = PIDController(desired_velocity - velocity);

        // Print the current velocity and throttle
        // RCLCPP_INFO(this->get_logger(), "Current velocity: %f", velocity);
        // RCLCPP_INFO(this->get_logger(), "Throttle: %f", throttle);

        return throttle;
    }

    float calculateSteeringAngle()
    {
        return 0.0;
    }

    float PIDController(float error)
    {
        float d_error = 0.0;
        float dt = 0.0;

        // Print error, current time, accum error, d error
        // RCLCPP_INFO(this->get_logger(), "Error: %f", error);
        // RCLCPP_INFO(this->get_logger(), "Current time: %f", this->now().seconds());
        // RCLCPP_INFO(this->get_logger(), "Accumulated error: %f", accum_error);
        // RCLCPP_INFO(this->get_logger(), "Differential error: %f", d_error);

        if (prev_time < 0.0)
        {
            // Set current time as previous time
            prev_time = this->now().seconds();
        } else {
            // Get current time
            float current_time = this->now().seconds();
            dt = current_time - prev_time;
            prev_time = current_time;
            accum_error += error * dt;
            d_error = (error - prev_error) / dt;
        }
        prev_error = error;

        return kP * error;

        // return kP * error + kI * accum_error + kD * d_error;
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_sub_;

    nav_msgs::msg::Odometry odom_msg_;

    // PID controller velocity
    float kP = 0.1;
    float kI = 0.0;
    float kD = 0.0;

    float accum_error = 0.0;
    float prev_error = 0.0;

    float prev_time = -1.0; // Initialize to -1 to indicate first iteration

    float wheel_base = .324; // m

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