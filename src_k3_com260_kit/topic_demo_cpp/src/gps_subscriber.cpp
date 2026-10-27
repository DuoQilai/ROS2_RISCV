#include <cmath>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

class GpsSubscriber : public rclcpp::Node
{
public:
  GpsSubscriber()
  : Node("gps_subscriber")
  {
    subscription_ = create_subscription<geometry_msgs::msg::Point>(
      "/gps_position", 10, std::bind(&GpsSubscriber::handle, this, std::placeholders::_1));
  }

private:
  void handle(const geometry_msgs::msg::Point::SharedPtr message)
  {
    const double distance = std::hypot(message->x, message->y);
    RCLCPP_INFO(
      get_logger(), "GPS_RECEIVED x=%.2f y=%.2f distance=%.2f", message->x, message->y, distance);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsSubscriber>());
  rclcpp::shutdown();
  return 0;
}
