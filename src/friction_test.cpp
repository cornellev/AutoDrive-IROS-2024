#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <fstream>

class FrictionTest : public rclcpp::Node
{
  public:
    FrictionTest() : Node("friction_test")
    {
      throttle_command_pub_ = create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command",10);
      timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&FrictionTest::publishCommands, this));
      left_wheel_rpm_sub_ = create_subscription<sensor_msgs::msg::JointState>("/autodrive/f1tenth_1/left_encoder",
                              10,std::bind(&FrictionTest::left_data, this, std::placeholders::_1));
      right_wheel_rpm_sub_ = create_subscription<sensor_msgs::msg::JointState>("/autodrive/f1tenth_1/right_encoder",
                              10,std::bind(&FrictionTest::right_data, this, std::placeholders::_1));
      ips_sub_ = create_subscription<geometry_msgs::msg::Point>("/autodrive/f1tenth_1/ips",
                              10,std::bind(&FrictionTest::ips_data,this,std::placeholders::_1));
      left_wheel_file_.open("../data/left_wheel_data.csv");
      right_wheel_file_.open("../data/right_wheel_data.csv");
      ips_file_.open("../data/ips_data.csv");
    }

    ~FrictionTest()
      {
        // Close files
        left_wheel_file_.close();
        right_wheel_file_.close();
        ips_file_.close();
      }

  private:
    void left_data(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      auto now = this->now();
      left_wheel_file_ << now.seconds() << "," << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << msg->velocity[0] << "\n";
    }

    void right_data(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      auto now = this->now();
      right_wheel_file_ << now.seconds() << "," << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << msg->velocity[0] << "\n";
    }

    void ips_data(const geometry_msgs::msg::Point::SharedPtr msg)
    {
      auto now = this->now();
      ips_file_ << now.seconds() << "," << msg->x << "," << msg->y << "," << msg->z << "\n";
    }

    void publishCommands()
    {
      auto throttle_msg = std_msgs::msg::Float32();
      std::cout << "flip throttle state\n\n\n\n\n\n\n\n\n\n\n\n\n" << std::endl;
      throttle_state_ = !throttle_state_;
      throttle_msg.data = throttle_state_ ? 1.0 : 0.0;
      throttle_command_pub_->publish(throttle_msg);
    }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_wheel_rpm_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_wheel_rpm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ips_sub_;

  std::ofstream left_wheel_file_;
  std::ofstream right_wheel_file_;
  std::ofstream ips_file_;

  bool throttle_state_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<FrictionTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}