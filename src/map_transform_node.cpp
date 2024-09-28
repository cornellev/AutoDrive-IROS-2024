#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "std_msgs/msg/float32.hpp"
#include <cmath>  // For trigonometry and std::floor
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <limits>
#include <control_toolbox/pid.hpp>

const double LIDAR_MAX = 10.0;

class MapTransformNode : public rclcpp::Node
{
public:
    MapTransformNode() : Node("map_transform_node")
    {
        // Subscriber for the OccupancyGrid /map topic
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapTransformNode::map_callback, this, std::placeholders::_1));

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        steering_pub_ = create_publisher<std_msgs::msg::Float32>("/control_target/steer_angle", 1);
        velocity_pub_ = create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 1);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        // Timer for periodically checking the transform between base_link and map
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MapTransformNode::timer_callback, this));

        pid_ = control_toolbox::Pid(0.1, 0.0, 0.1);
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_data_ = *msg;
        RCLCPP_DEBUG(this->get_logger(), "Map received and saved.");
    }

    // Function to lookup the transform from base_link to map and perform raytracing
    void timer_callback()
    {
        if (!map_data_.info.resolution) {
            RCLCPP_WARN(this->get_logger(), "No map data available.");
            // drive a little so we get map data
            auto velocity_msg = std_msgs::msg::Float32();
            velocity_msg.data = .01;
            velocity_pub_->publish(velocity_msg);

            return;
        }

        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped;
            // Lookup the transform from "map" to "base_link"
            transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            // Get the base_link position and yaw in the map frame
            double base_link_x = transform_stamped.transform.translation.x;
            double base_link_y = transform_stamped.transform.translation.y;

            // Get yaw from the quaternion
            double yaw = get_yaw_from_quaternion(transform_stamped.transform.rotation);

            // Print the entire map grid in ascii
            // for (size_t i = 0; i < map_data_.info.height; i++)
            // {
            //     for (size_t j = 0; j < map_data_.info.width; j++)
            //     {
            //         int index = i * map_data_.info.width + j;
            //         if (map_data_.data[index] == 0)
            //         {
            //             std::cout << ".";
            //         }
            //         else if (map_data_.data[index] == 100)
            //         {
            //             std::cout << "#";
            //         }
            //         else
            //         {
            //             std::cout << " ";
            //         }
            //     }
            //     std::cout << std::endl;
            // }

            // Raytrace in directions: yaw ± pi/4, yaw ± pi/3, and yaw ± pi/2
            double directions[8] = {
                yaw + M_PI / 2.,
                yaw + M_PI / 2.5,
                yaw + M_PI / 4.,
                yaw + M_PI / 6.,
                yaw - M_PI / 2.,   // yaw + pi/2,
                yaw - M_PI / 2.5,   // yaw + pi/2
                yaw - M_PI / 4.,  // yaw + pi/4
                yaw - M_PI / 6.  // yaw + pi/6
            };

            double distances[8];

            // Perform raytracing for each direction
            for (size_t i = 0; i < sizeof(directions) / sizeof(double); i++) {
                double direction = directions[i];
                double distance = raytrace(base_link_x, base_link_y, direction);
                
                if (distance > 10.0) {
                    distance = 10.0;
                }
                
                // if (distance <= 10.0) {
                //     // RCLCPP_DEBUG(this->get_logger(),
                //     //             "Obstacle found at distance %.2f in direction %.2f radians",
                //     //             distance, direction);
                // } else {
                //     // RCLCPP_DEBUG(this->get_logger(), "No obstacle found in direction %.2f radians", direction);
                // }
                distances[i] = distance;
            }

            // Farthest dist, angle
            auto [best_angle, best_dist] = find_farthest_angle(base_link_x, base_link_y, yaw);

            publish_direction_marker(base_link_x, base_link_y, best_angle - yaw, best_dist);

            // If s is more than the lidar's farthest distance, set it to the farthest distance
            if (best_dist > LIDAR_MAX)
            {
                best_dist = LIDAR_MAX;
            }

            double wheel_base = 0.324;  // m

            // Calculate steering angle based on ackermann steering model
            float steering_angle = atan(3.9 * wheel_base * sin(best_angle - yaw) / best_dist);

            double dist_left[4] = { distances[0], distances[1], distances[2], distances[3] };
            double dist_right[4] = { distances[4], distances[5], distances[6], distances[7] };


            // RCLCPP_INFO(this->get_logger(), "Dist left: %f, dist right: %f", dist_left[0], dist_right[0]);

            auto steering_msg = std_msgs::msg::Float32();

            // RCLCPP_INFO(this->get_logger(), "Searching at yaw angle %.2f", yaw);
            // Print the yaw in the occupancy grid info message
            // RCLCPP_INFO(this->get_logger(), "Map yaw: %.2f", get_yaw_from_quaternion(map_data_.info.origin.orientation));

            steering_msg.data = steering_angle;

            // steering_msg.data = 0.0;

            // if (dist_left[0] > 0.0) {
            //     steering_msg.data -= (M_PI / 8.5) * dist_left[0];
            // }

            // if (dist_right[0] > 0.0) {
            //     steering_msg.data += (M_PI / 8.5) * dist_right[0];
            // }

            // steering_msg.data = (M_PI / 9.0) * (dist_left[0] - dist_right[0]);
            // steering_msg.data += (M_PI / 10.0) * (dist_left[1] - dist_right[1]);
            // steering_msg.data += (M_PI / 10.3) * (dist_left[2] - dist_right[2]);
            // steering_msg.data += (M_PI / 11.3) * (dist_left[3] - dist_right[3]);

            // if (steering_msg.data > M_PI / 5.0) {
            //     steering_msg.data = M_PI / 5.0;
            // } else if (steering_msg.data < -M_PI / 5.0) {
            //     steering_msg.data = -M_PI / 5.0;
            // }

            double avg_dist = (dist_left[0] + dist_right[0]) / 2.0;

            double ratio = avg_dist / 1.3;

            // if (dist_left[1] < 0.8) {
            //     steering_msg.data -= avg_dist * (M_PI / 8.0) * (0.8 - dist_left[1]);
            // }

            // if (dist_right[1] < 0.8) {
            //     steering_msg.data += avg_dist * (M_PI / 8.0) * (0.8 - dist_right[1]);
            // }

            // START PID
            if (!last_time_.has_value()) {
                last_time_ = rclcpp::Time(transform_stamped.header.stamp);
                return;
            }
            rclcpp::Time now(transform_stamped.header.stamp);
            rclcpp::Duration dt = now - last_time_.value();
            double error = (dist_left[0] + dist_left[3]) - (dist_right[0] + dist_right[3]);
            double pid_control = pid_.computeCommand(error, dt.nanoseconds());

            steering_msg.data += 0.5 * pid_control;
            // END PID

            RCLCPP_INFO(this->get_logger(), "Steering angle: %.2f", steering_msg.data * 180.0 / M_PI);
            steering_pub_->publish(steering_msg);
 
            auto velocity_msg = std_msgs::msg::Float32();
            velocity_msg.data = .01;
            velocity_pub_->publish(velocity_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
        }
    }

    // Helper function to convert quaternion to yaw (heading in radians)
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
    {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    double raytrace(double start_x, double start_y, double direction)
    {
        double resolution = map_data_.info.resolution;
        int width = map_data_.info.width;
        int height = map_data_.info.height;
        double origin_x = map_data_.info.origin.position.x;
        double origin_y = map_data_.info.origin.position.y;

        // Starting position in grid coordinates
        int grid_start_x = static_cast<int>((start_x - origin_x) / resolution);
        int grid_start_y = static_cast<int>((start_y - origin_y) / resolution);

        // Check if the starting position is within map bounds
        if (grid_start_x < 0 || grid_start_x >= width || grid_start_y < 0 || grid_start_y >= height) {
            return std::numeric_limits<double>::infinity();  // Out of bounds
        }

        // Ray step increments in the x and y directions
        double step_size = 0.05;  // Increment size along the ray (in meters)
        double ray_x = start_x, ray_y = start_y;

        // Trace along the ray
        while (true)
        {
            // Move along the ray in small steps
            ray_x += step_size * std::cos(direction);
            ray_y += step_size * std::sin(direction);

            // Convert current position to grid coordinates
            int grid_x = static_cast<int>((ray_x - origin_x) / resolution);
            int grid_y = static_cast<int>((ray_y - origin_y) / resolution);

            // Check if the position is within map bounds
            if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
                return std::numeric_limits<double>::infinity();  // No obstacle found (out of bounds)
            }

            // Get the occupancy value of the current grid cell
            int index = grid_y * width + grid_x;

            int occupancy_value = map_data_.data[index];

            // Check if the cell is occupied (obstacle)
            if (occupancy_value >= 50) {  // Adjust this threshold as needed
                // Print found cell
                ray_x -= start_x;
                ray_y -= start_y;

                // RCLCPP_INFO(this->get_logger(), "Obstacle found at (%d, %d) with dist %f", grid_x, grid_y, std::sqrt(ray_x * ray_x + ray_y * ray_y));
                return std::sqrt(ray_x * ray_x + ray_y * ray_y);  // Obstacle found
            }
        }

        return std::numeric_limits<double>::infinity();  // No obstacle found
    }


    std::pair<double, double> find_farthest_angle(double start_x, double start_y, double yaw)
    {
        double best_angle = 0.0;
        double farthest_distance = 0.0;
        double distance_threshold = 0.1;  // 0.1 meter threshold

        // Iterate over angles from -π/2 to π/2, sweeping in front of the vehicle
        double angle_step = M_PI / 180.0;  // 1 degree in radians
        double min_angle = yaw - M_PI_2;   // -90 degrees relative to yaw
        double max_angle = yaw + M_PI_2;   // +90 degrees relative to yaw

        for (double angle = min_angle; angle <= max_angle; angle += angle_step)
        {
            // Perform raytrace for the current angle and get the distance
            double distance = raytrace(start_x, start_y, angle);

            // Check if the current distance is greater or within the threshold
            // if (distance > farthest_distance ||
            //     (std::abs(distance - farthest_distance) < distance_threshold && std::abs(angle - yaw) < std::abs(best_angle - yaw))) {
            //     best_angle = angle;
            //     farthest_distance = distance;
            // }
            if (distance > farthest_distance) {
                best_angle = angle;
                farthest_distance = distance;
            }
        }

        // Return the best angle (in radians) and the farthest distance
        return {best_angle, farthest_distance};
    }

    void publish_direction_marker(double start_x, double start_y, double angle, double distance)
    {
        // Create a marker message
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Publish relative to map frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "direction_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Define the arrow scale (size)
        marker.scale.x = 0.01;      // Arrow length (the magnitude of the distance)
        marker.scale.y = 0.01;      // Arrow width
        marker.scale.z = 0.01;      // Arrow height

        // Define the arrow color (RGBA)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // Find the base_link position relative to map
        geometry_msgs::msg::TransformStamped base_to_map_tf;
        try {
            base_to_map_tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        // Define the start point of the arrow (base_link origin in the map frame)
        geometry_msgs::msg::Point start_point;
        start_point.x = base_to_map_tf.transform.translation.x;
        start_point.y = base_to_map_tf.transform.translation.y;
        start_point.z = base_to_map_tf.transform.translation.z;

        // Compute the end point based on the angle in the map frame
        geometry_msgs::msg::Point end_point;
        end_point.x = start_point.x + distance * std::cos(angle);
        end_point.y = start_point.y + distance * std::sin(angle);
        end_point.z = start_point.z;  // Keep the z level the same

        // Set the points for the arrow
        marker.points.push_back(start_point);  // Start at base_link
        marker.points.push_back(end_point);    // Point in the direction of the chosen angle

        // Publish the marker
        marker_pub_->publish(marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;

    // Member variables
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    nav_msgs::msg::OccupancyGrid map_data_;  // Store the map data
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    control_toolbox::Pid pid_;
    std::optional<rclcpp::Time> last_time_;
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapTransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
