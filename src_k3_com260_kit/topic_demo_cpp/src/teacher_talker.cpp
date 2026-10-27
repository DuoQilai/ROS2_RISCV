#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TalkerNode : public rclcpp::Node
{
public:
  TalkerNode() : Node("talker")
  {
    publisher_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      std_msgs::msg::String message;
      message.data = "Hello ROS 2: " + std::to_string(count_++);
      publisher_->publish(message);
      RCLCPP_INFO(get_logger(), "发布: %s", message.data.c_str());
    });
  }
private:
  int count_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalkerNode>());
  rclcpp::shutdown();
  return 0;
}
