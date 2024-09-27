#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"

class MapTransformNode : public rclcpp::Node
{
public:
    MapTransformNode() : Node("map_transform_node")
    {
        // Subscriber for the OccupancyGrid /map topic
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 1, std::bind(&MapTransformNode::map_callback, this, std::placeholders::_1));

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer for periodically checking the transform between base_link and map
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MapTransformNode::lookup_transform, this));
    }

private:
    // Callback for receiving the map
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_data_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Map received and saved.");
    }

    // Function to lookup the transform from base_link to map
    void lookup_transform()
    {
        if (!map_data_.info.resolution) {
            RCLCPP_WARN(this->get_logger(), "No map data available.");
            return;
        }

        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped;

            transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            // Get the base_link position in the map frame
            double base_link_x = transform_stamped.transform.translation.x;
            double base_link_y = transform_stamped.transform.translation.y;

            // Get map information
            double map_origin_x = map_data_.info.origin.position.x;
            double map_origin_y = map_data_.info.origin.position.y;
            double resolution = map_data_.info.resolution;

            // Calculate grid cell coordinates
            int cell_x = static_cast<int>(std::floor((base_link_x - map_origin_x) / resolution));
            int cell_y = static_cast<int>(std::floor((base_link_y - map_origin_y) / resolution));

            // Check if the coordinates are within the map bounds
            if (cell_x >= 0 && cell_x < static_cast<int>(map_data_.info.width) &&
                cell_y >= 0 && cell_y < static_cast<int>(map_data_.info.height))
            {
                RCLCPP_INFO(this->get_logger(),
                            "Base link position in grid: (x: %d, y: %d)", cell_x, cell_y);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Base link position is outside of the map bounds.");
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
        }
    }

    // Member variables
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    nav_msgs::msg::OccupancyGrid map_data_;  // Store the map data
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapTransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}