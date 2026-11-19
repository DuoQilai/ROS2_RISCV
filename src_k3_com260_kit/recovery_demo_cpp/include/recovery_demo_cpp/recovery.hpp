#pragma once
#include <cmath>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/spin.hpp>
#include "recovery_demo_cpp/action_client.hpp"

inline bool spin(const rclcpp::Node::SharedPtr & node, double yaw = 1.57, double seconds = 5) {
  if (!std::isfinite(yaw) || !std::isfinite(seconds) || seconds <= 0) {
    RCLCPP_ERROR(node->get_logger(), "旋转角度必须有限，允许时间必须为正数");
    return false;
  }
  nav2_msgs::action::Spin::Goal goal;
  goal.target_yaw = yaw;
  goal.time_allowance = rclcpp::Duration::from_seconds(seconds);
  return run_action<nav2_msgs::action::Spin>(node, "/spin", goal);
}

inline bool backup(const rclcpp::Node::SharedPtr & node, double distance = 0.15, double speed = 0.05) {
  if (!std::isfinite(distance) || !std::isfinite(speed) || distance <= 0 || speed <= 0) {
    RCLCPP_ERROR(node->get_logger(), "后退距离和速度必须为有限正数");
    return false;
  }
  nav2_msgs::action::BackUp::Goal goal;
  goal.target.x = -distance;
  goal.speed = speed;
  goal.time_allowance = rclcpp::Duration::from_seconds(distance / speed + 1);
  return run_action<nav2_msgs::action::BackUp>(node, "/backup", goal);
}

inline bool wait(const rclcpp::Node::SharedPtr & node, double seconds) {
  RCLCPP_INFO(node->get_logger(), "等待 %.1f s", seconds);
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (!stop_requested && std::chrono::steady_clock::now() < end) {
    rclcpp::sleep_for(20ms);
  }
  return !stop_requested;
}
