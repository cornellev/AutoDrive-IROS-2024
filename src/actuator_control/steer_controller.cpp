#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

const uint32_t TIMER_MS = 20;
const double MAX_ANGLE_RAD = 0.5236;

class SteerController : public rclcpp::Node
{
public:
    SteerController() : Node("steer_controller")
    {
        auto feedback_topic = declare_parameter<std::string>("feedback_topic", "/autodrive/f1tenth_1/steering");
        auto target_topic = declare_parameter<std::string>("target_topic", "/control_target/steer_angle");
        auto out_topic = declare_parameter<std::string>("out_topic", "/autodrive/f1tenth_1/steering_command");

        auto keep_last = rclcpp::QoS(rclcpp::KeepLast(1));

        // important: all callbacks will be called sequentially, never in parallel
        // if this weren't true, current_steer_ could change from under us in timer
        feedback_sub_ = create_subscription<std_msgs::msg::Float32>(
            feedback_topic, keep_last,
            std::bind(&SteerController::feedback_callback, this, std::placeholders::_1)
        );

        target_sub_ = create_subscription<std_msgs::msg::Float32>(
            target_topic, keep_last,
            std::bind(&SteerController::target_callback, this, std::placeholders::_1)
        );

        publisher_ = create_publisher<std_msgs::msg::Float32>(
            out_topic, keep_last
        );

        timer_ = create_wall_timer(std::chrono::milliseconds(TIMER_MS), std::bind(&SteerController::timer, this));

        steer_velocity_ = declare_parameter<double>("steer_velocity", 0.1);

        RCLCPP_INFO(get_logger(), "Started steering controller.");
    }

private:
    void timer()
    {
        float diff = target_steer_ - current_steer_;
        float change = steer_velocity_ * (TIMER_MS / 1000.0);
        if (abs(diff) < change) {
            current_steer_ = target_steer_;
        } else {
            if (diff > 0) {
                current_steer_ += change;
            } else {
                current_steer_ -=change;
            }
        }

        if (current_steer_ > MAX_ANGLE_RAD) {
            current_steer_ = MAX_ANGLE_RAD;
        } else if (current_steer_ < -MAX_ANGLE_RAD) {
            current_steer_ = -MAX_ANGLE_RAD;
        }

        std_msgs::msg::Float32 msg;
        msg.data = current_steer_ / MAX_ANGLE_RAD;
        publisher_->publish(msg);
    }

    void feedback_callback(const std_msgs::msg::Float32::ConstSharedPtr current_steer)
    {
        current_steer_ = current_steer->data;
    }

    void target_callback(const std_msgs::msg::Float32::ConstSharedPtr target) 
    {
        target_steer_ = target->data / max_steering_angle;
    }

    float steer_velocity_ = 0.0;
    float target_steer_ = 0.0;
    float current_steer_ = 0.0;

    float max_steering_angle = 0.5236;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedback_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteerController>());
    rclcpp::shutdown();
    return 0;
}
