#include <cmath>
#include <functional>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class OdomMonitor : public rclcpp::Node
{
public:
  OdomMonitor()
  : Node("odom_monitor")
  {
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&OdomMonitor::handle, this, std::placeholders::_1));
  }

private:
  void handle(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    const auto & orientation = message->pose.pose.orientation;
    const auto & velocity = message->twist.twist;
    const double yaw = std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
    RCLCPP_INFO(
      get_logger(), "ODOM_RECEIVED x=%.2f m y=%.2f m yaw=%.2f rad vx=%.3f m/s wz=%.3f rad/s",
      position.x, position.y, yaw, velocity.linear.x, velocity.angular.z);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomMonitor>());
  rclcpp::shutdown();
  return 0;
}
