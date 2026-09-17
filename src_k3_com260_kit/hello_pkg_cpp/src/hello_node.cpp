#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class HelloNode : public rclcpp::Node
{
public:
  HelloNode()
  : Node("hello_node"), count_(0)
  {
    timer_ = create_wall_timer(1s, std::bind(&HelloNode::tick, this));
    RCLCPP_INFO(get_logger(), "HELLO_CPP_OK HelloNode started");
  }

private:
  void tick()
  {
    ++count_;
    RCLCPP_INFO(get_logger(), "Hello ROS 2! count=%zu", count_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloNode>());
  rclcpp::shutdown();
  return 0;
}
