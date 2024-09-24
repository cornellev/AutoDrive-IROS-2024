#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class FrictionTest : public rclcpp::Node
{
  public:
    FrictionTest() : Node("friction_test")
    {
      throttle_pub_ = create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command",10);
      timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&FrictionTest::publishCommands, this));
    }
  private:
    void publishCommands()
    {
      auto throttle_msg = std_msgs::msg::Float32();
      this->now().seconds() > 2 ? throttle_msg.data = 1 : throttle_msg.data = 0;
      throttle_pub_->publish(throttle_msg);
    }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float prev_time = -1.0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<FrictionTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}