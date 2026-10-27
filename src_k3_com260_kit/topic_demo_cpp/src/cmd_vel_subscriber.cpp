#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class VelocitySubscriber : public rclcpp::Node
{
public:
  VelocitySubscriber() : Node("velocity_subscriber")
  {
    subscription_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        RCLCPP_INFO(get_logger(), "linear.x=%.2f m/s angular.z=%.2f rad/s",
          message->linear.x, message->angular.z);
      });
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocitySubscriber>());
  rclcpp::shutdown();
  return 0;
}
