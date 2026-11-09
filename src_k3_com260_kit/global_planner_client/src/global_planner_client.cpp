#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace std::chrono_literals;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("global_planner_client");
  using Plan = nav2_msgs::action::ComputePathToPose;
  auto client = rclcpp_action::create_client<Plan>(node, "/compute_path_to_pose");
  auto publisher = node->create_publisher<nav_msgs::msg::Path>("/plan", rclcpp::QoS(1).transient_local());
  Plan::Goal goal;
  goal.start.header.frame_id = goal.goal.header.frame_id = "map";
  goal.start.header.stamp = goal.goal.header.stamp = node->now();
  goal.start.pose.position.x = node->declare_parameter("start_x", 0.0);
  goal.start.pose.position.y = node->declare_parameter("start_y", 0.0);
  goal.start.pose.orientation.w = goal.goal.pose.orientation.w = 1;
  goal.goal.pose.position.x = node->declare_parameter("goal_x", 3.0);
  goal.goal.pose.position.y = node->declare_parameter("goal_y", 2.0);
  goal.use_start = true;
  goal.planner_id = node->declare_parameter("planner_id", "GridBased");
  for (double value : {goal.start.pose.position.x, goal.start.pose.position.y,
      goal.goal.pose.position.x, goal.goal.pose.position.y}) {
    if (!std::isfinite(value)) {
      RCLCPP_ERROR(node->get_logger(), "规划坐标必须为有限数值");
      rclcpp::shutdown();
      return 2;
    }
  }
  bool success = false;
  if (client->wait_for_action_server(15s)) {
    auto sent = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node, sent, 10s) == rclcpp::FutureReturnCode::SUCCESS && sent.get()) {
      auto result = client->async_get_result(sent.get());
      if (rclcpp::spin_until_future_complete(node, result, 20s) == rclcpp::FutureReturnCode::SUCCESS) {
        const auto reply = result.get();
        success = reply.code == rclcpp_action::ResultCode::SUCCEEDED && !reply.result->path.poses.empty();
        if (success) {
          publisher->publish(reply.result->path);
          RCLCPP_INFO(node->get_logger(), "路径规划成功: %zu 个路径点", reply.result->path.poses.size());
          rclcpp::sleep_for(1s);
        }
      }
    }
  }
  if (!success) { RCLCPP_ERROR(node->get_logger(), "路径规划失败或超时"); }
  rclcpp::shutdown();
  return success ? 0 : 1;
}
