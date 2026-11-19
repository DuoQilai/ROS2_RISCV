#include <cmath>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "recovery_demo_cpp/action_client.hpp"
int main(int argc, char ** argv) {
  init_actions(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_client");
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node->now();
  goal.pose.pose.position.x = node->declare_parameter("x", 0.0);
  goal.pose.pose.position.y = node->declare_parameter("y", 0.0);
  const double yaw = node->declare_parameter("yaw", 0.0);
  goal.pose.pose.orientation.z = std::sin(yaw / 2);
  goal.pose.pose.orientation.w = std::cos(yaw / 2);
  goal.behavior_tree = node->declare_parameter<std::string>("behavior_tree", "");
  const bool ok = run_action<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose", goal, 180);
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
