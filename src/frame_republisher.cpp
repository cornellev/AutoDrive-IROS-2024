#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class FrameRepublisher : public rclcpp::Node
{
public:
    FrameRepublisher() : Node("frame_republisher")
    {
        // Get the list of frames to republish
        this->declare_parameter<std::vector<std::string>>("frames", std::vector<std::string>{});
        
        this->get_parameter("frames", frames_);

        // Initialize TF listener and broadcaster
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Set timer to periodically republish the frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&FrameRepublisher::republishFrames, this));
    }

private:
    void republishFrames()
    {
        for (const auto &frame : frames_)  // Transform every frame in the list
        {
            try
            {
                // Get the transformation from the source frame to the target, time 0 means latest
                geometry_msgs::msg::TransformStamped transformStamped = 
                    tf_buffer_->lookupTransform("f1tenth_1", frame, rclcpp::Time(0));

                // Publish the same transformation with base_link as the new parent frame
                geometry_msgs::msg::TransformStamped newTransformStamped = transformStamped;

                newTransformStamped.child_frame_id = frame + "_republished"; // Append _republished
                newTransformStamped.header.frame_id = "base_link"; // Attach to base_link

                tf_broadcaster_->sendTransform(newTransformStamped);

                // RCLCPP_INFO(this->get_logger(), "Republished %s to base_link", frame.c_str());
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not transform %s to base_link: %s", (frame + "_republished").c_str(), ex.what());
            }
        }
    }

    std::vector<std::string> frames_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrameRepublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
