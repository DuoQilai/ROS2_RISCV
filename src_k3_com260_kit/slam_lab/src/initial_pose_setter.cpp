#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  auto node = std::make_shared<rclcpp::Node>("initial_pose_setter");
  try {
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() != 4) {throw std::invalid_argument("Usage: set_initial_pose X Y YAW_DEG [--ros-args ...]");}
    const auto number = [](const std::string & value) {
        size_t used = 0;
        const double result = std::stod(value, &used);
        if (used != value.size() || !std::isfinite(result)) {throw std::invalid_argument("Pose values must be finite numbers");}
        return result;
      };
    const double x = number(args[1]), y = number(args[2]), yaw_deg = number(args[3]);
    const double yaw = yaw_deg * std::acos(-1.0) / 180.0;
    auto pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (rclcpp::ok() && pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(100ms);
    }
    if (pub->get_subscription_count() == 0) {
      RCLCPP_WARN(node->get_logger(), "No /initialpose subscriber found; check whether AMCL is active");
    }
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.0685;
    for (int i = 0; i < 3 && rclcpp::ok(); ++i) {
      msg.header.stamp = node->now();
      pub->publish(msg);
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(200ms);
    }
    RCLCPP_INFO(node->get_logger(), "Published initial pose: x=%.2f, y=%.2f, yaw=%.1f deg", x, y, yaw_deg);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    status = 2;
  }
  rclcpp::shutdown();
  return status;
}
