#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PositionPublisher : public rclcpp::Node
{
public:
  PositionPublisher()
  : Node("gps_publisher"), x_(0.0)
  {
    publisher_ = create_publisher<geometry_msgs::msg::Point>("/gps_position", 10);
    timer_ = create_wall_timer(1s, std::bind(&PositionPublisher::publish, this));
  }

private:
  void publish()
  {
    geometry_msgs::msg::Point message;
    message.x = x_;
    message.y = 2.0 * x_ + 1.0;
    publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "GPS_PUBLISHED x=%.2f y=%.2f", message.x, message.y);
    x_ += 1.0;
  }

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double x_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionPublisher>());
  rclcpp::shutdown();
  return 0;
}
