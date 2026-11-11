#include <chrono>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "dwa_planner/dwa.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stop_requested = 0;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) { stop_requested = 1; });
  std::signal(SIGTERM, [](int) { stop_requested = 1; });
  auto node = std::make_shared<rclcpp::Node>("dwa_controller");
  DWAConfig config;
  config.max_speed = node->declare_parameter("max_speed", 0.15);
  const double timeout = node->declare_parameter("timeout", 180.0);
  if (!std::isfinite(config.max_speed) || config.max_speed <= 0 ||
      !std::isfinite(timeout) || timeout <= 0) {
    RCLCPP_ERROR(node->get_logger(), "速度和超时必须为正数");
    rclcpp::shutdown(); return 2;
  }
  tf2_ros::Buffer transforms(node->get_clock());
  tf2_ros::TransformListener listener(transforms);
  Motion state{};
  Path obstacles, path;
  Clock::time_point last_odom{}, last_scan{};
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto odom = node->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(),
    [&](nav_msgs::msg::Odometry::ConstSharedPtr message) {
      if (message->header.frame_id != "odom") { return; }
      state = {message->pose.pose.position.x, message->pose.pose.position.y,
        tf2::getYaw(message->pose.pose.orientation),
        message->twist.twist.linear.x, message->twist.twist.angular.z};
      last_odom = Clock::now();
    });
  auto scan = node->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
      try {
        auto transform = transforms.lookupTransform("odom", message->header.frame_id, tf2::TimePointZero);
        Path points;
        for (std::size_t i = 0; i < message->ranges.size(); ++i) {
          const double range = message->ranges[i];
          if (!std::isfinite(range) || range < message->range_min || range > message->range_max) { continue; }
          const double angle = message->angle_min + i * message->angle_increment;
          geometry_msgs::msg::PointStamped point, converted;
          point.point.x = range * std::cos(angle);
          point.point.y = range * std::sin(angle);
          tf2::doTransform(point, converted, transform);
          points.push_back({converted.point.x, converted.point.y});
        }
        obstacles = std::move(points);
        last_scan = Clock::now();
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "%s", error.what());
      }
    });
  auto plan = node->create_subscription<nav_msgs::msg::Path>("plan", 10,
    [&](nav_msgs::msg::Path::ConstSharedPtr message) {
      try {
        auto transform = transforms.lookupTransform("odom", message->header.frame_id, tf2::TimePointZero);
        Path points;
        for (const auto & pose : message->poses) {
          geometry_msgs::msg::PointStamped point, converted;
          point.point = pose.pose.position;
          tf2::doTransform(point, converted, transform);
          points.push_back({converted.point.x, converted.point.y});
        }
        path = std::move(points);
        RCLCPP_INFO(node->get_logger(), "收到路径: %zu 点", path.size());
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN(node->get_logger(), "%s", error.what());
      }
    });
  const auto begin = Clock::now();
  bool arrived = false;
  while (rclcpp::ok() && !stop_requested && std::chrono::duration<double>(Clock::now() - begin).count() < timeout) {
    rclcpp::spin_some(node);
    geometry_msgs::msg::Twist command;
    if (!path.empty() && Clock::now() - last_odom < 500ms && Clock::now() - last_scan < 500ms) {
      const auto goal = path.back();
      arrived = std::hypot(goal.first - state.x, goal.second - state.y) <= 0.1;
      if (arrived) { break; }
      const auto result = dwa_plan(state, goal, obstacles, path, config);
      command.linear.x = result.command.v;
      command.angular.z = result.command.w;
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
        "v=%.3f w=%.3f 距离=%.3f", command.linear.x, command.angular.z,
        std::hypot(goal.first - state.x, goal.second - state.y));
    }
    publisher->publish(command);
    rclcpp::sleep_for(100ms);
  }
  for (int i = 0; i < 10 && rclcpp::ok(); ++i) {
    publisher->publish(geometry_msgs::msg::Twist());
    rclcpp::sleep_for(50ms);
  }
  const double remaining = path.empty() ? std::numeric_limits<double>::infinity() :
    std::hypot(path.back().first - state.x, path.back().second - state.y);
  RCLCPP_INFO(node->get_logger(), "%s，odom 路径终点距离=%.4f m",
    arrived ? "到达目标，已发送停止" : "取消或超时，已发送停止", remaining);
  rclcpp::shutdown();
  return arrived ? 0 : 1;
}
