#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ListenerNode : public rclcpp::Node
{
public:
  ListenerNode() : Node("listener")
  {
    subscription_ = create_subscription<std_msgs::msg::String>("chatter", 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        RCLCPP_INFO(get_logger(), "收到: %s", message->data.c_str());
      });
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ListenerNode>());
  rclcpp::shutdown();
  return 0;
}
