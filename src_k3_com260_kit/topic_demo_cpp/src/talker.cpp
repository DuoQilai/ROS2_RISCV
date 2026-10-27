#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "topic_demo_interfaces/msg/gps.hpp"

using namespace std::chrono_literals;

class GpsTalker : public rclcpp::Node
{
public:
  GpsTalker()
  : Node("pytalker"), x_(1.0F), y_(2.0F), state_("working")
  {
    publisher_ = create_publisher<topic_demo_interfaces::msg::Gps>("gps_info", 10);
    timer_ = create_wall_timer(1s, std::bind(&GpsTalker::publish_message, this));
  }

private:
  void publish_message()
  {
    topic_demo_interfaces::msg::Gps msg;
    msg.state = state_;
    msg.x = x_;
    msg.y = y_;
    RCLCPP_INFO(get_logger(), "Talker: GPS: x=%.6f, y=%.6f", msg.x, msg.y);
    publisher_->publish(msg);
    x_ *= 1.03F;
    y_ *= 1.01F;
  }

  rclcpp::Publisher<topic_demo_interfaces::msg::Gps>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  float x_;
  float y_;
  std::string state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsTalker>());
  rclcpp::shutdown();
  return 0;
}
