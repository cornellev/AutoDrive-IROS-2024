#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

class SimpleDriver : public rclcpp::Node
{
public:
    SimpleDriver()
        : Node("simple_driver")
    {
        steering_pub_ = create_publisher<std_msgs::msg::Float32>("/control_target/steer_angle", 1);
        // velocity_pub_ = create_publisher<std_msgs::msg::Float32>("/control_target/velocity", 10);
        velocity_pub_ = create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 1);

        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/autodrive/f1tenth_1/lidar", 1, 
            std::bind(&SimpleDriver::lidar_callback, this, std::placeholders::_1)
        );

        // Parameters
        car_length_ = 0.324; // wheelbase (in meters)
        min_gap_threshold_ = .5; // minimum gap distance (in meters)
    }

private:
    // Subscriber and Publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;

    // Parameters
    double car_length_;
    double min_gap_threshold_;

    // Lidar callback
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Preprocess lidar data
        std::vector<float> ranges = preprocess_lidar(msg->ranges);

        // Find the largest gap in the processed ranges
        std::vector<int> largest_gap = find_largest_gap(ranges);

        // Find the best point in the gap
        int best_point_idx = find_best_point(largest_gap, ranges.size());

        // Compute the optimal steering angle
        double steering_angle = compute_steering_angle(best_point_idx, msg->angle_min, msg->angle_increment);

        // Publish the steering command
        publish_drive_command(steering_angle);
    }

    // Preprocess the lidar scan to remove invalid values and smooth the data
    std::vector<float> preprocess_lidar(const std::vector<float>& ranges)
    {
        std::vector<float> processed_ranges = ranges;
        std::replace_if(processed_ranges.begin(), processed_ranges.end(),
                        [](float r) { return std::isinf(r) || std::isnan(r); }, 0.0f);

        // Apply a moving average filter (window size = 5)
        int window_size = 5;
        std::vector<float> smoothed_ranges(processed_ranges.size(), 0.0f);
        for (size_t i = 0; i < processed_ranges.size(); ++i) {
            float sum = 0.0;
            int count = 0;
            for (int j = -window_size / 2; j <= window_size / 2; ++j) {
                if (i + j >= 0 && i + j < processed_ranges.size()) {
                    sum += processed_ranges[i + j];
                    ++count;
                }
            }
            smoothed_ranges[i] = sum / count;
        }
        return smoothed_ranges;
    }

    // Find the largest gap in the lidar scan (a continuous sequence of valid readings)
    std::vector<int> find_largest_gap(const std::vector<float>& ranges)
    {
        std::vector<int> largest_gap;
        std::vector<int> current_gap;

        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] > min_gap_threshold_) {
                current_gap.push_back(i);
            } else {
                if (current_gap.size() > largest_gap.size()) {
                    largest_gap = current_gap;
                }
                current_gap.clear();
            }
        }

        if (current_gap.size() > largest_gap.size()) {
            largest_gap = current_gap;
        }

        return largest_gap;
    }

    // Find the best point in the largest gap (usually the midpoint)
    int find_best_point(const std::vector<int>& gap, size_t total_size)
    {
        if (gap.empty()) {
            return total_size / 2;  // Default to straight ahead if no gap
        }
        return gap[gap.size() / 2];
    }

    // Compute the steering angle to aim for the selected point
    double compute_steering_angle(int best_point_idx, float angle_min, float angle_increment)
    {
        return angle_min + best_point_idx * angle_increment;
    }

    // Publish the AckermannDrive command
    void publish_drive_command(double steering_angle)
    {
        auto velocity_msg = std_msgs::msg::Float32();
        velocity_msg.data = .01;
        velocity_pub_->publish(velocity_msg);

        auto steering_msg = std_msgs::msg::Float32();
        steering_msg.data = steering_angle;
        steering_pub_->publish(steering_msg);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleDriver>());
    rclcpp::shutdown();
    return 0;
}
