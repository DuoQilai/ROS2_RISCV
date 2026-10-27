#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pose_nav_interfaces/action/move_base.hpp"

using namespace std::chrono_literals;
using Action = pose_nav_interfaces::action::MoveBase;
int main(int argc, char ** argv)
{
  double x = 0.0, y = 0.0, third = 0.0, cancel_after = 0.0;
  try {
    auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() < 4 || args.size() > 5) {throw std::invalid_argument("用法：ros2 run pose_nav_action nav_client x y yaw [cancel_after]");}
    for (size_t i = 1; i < args.size(); ++i) {
      size_t used = 0;
      double value = std::stod(args[i], &used);
      if (used != args[i].size() || !std::isfinite(value)) {throw std::invalid_argument("坐标必须为有限数字");}
      if (i == 1) {x = value;} else if (i == 2) {y = value;}
      else if (i == 3) {third = value;} else {cancel_after = value;}
    }
    if (args.size() == 5 && cancel_after <= 0.0) {throw std::invalid_argument("取消时间必须大于零");}
  } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 2;}
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pose_nav_client");
  auto client = rclcpp_action::create_client<Action>(node, "move_base_lab");
  int exit_code = 1;
  if (client->wait_for_action_server(10s)) {
    Action::Goal goal;
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = node->now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = std::sin(third / 2.0);
    goal.target_pose.pose.orientation.w = std::cos(third / 2.0);
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.feedback_callback = [node](auto, const auto feedback) {
      RCLCPP_INFO(node->get_logger(), "distance_remaining=%.6f m", feedback->distance_remaining);
    };
    auto sent = client->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(node, sent, 10s) == rclcpp::FutureReturnCode::SUCCESS) {
      auto handle = sent.get();
      if (!handle) {RCLCPP_WARN(node->get_logger(), "目标被 Server 拒绝");}
      else {
        RCLCPP_INFO(node->get_logger(), "目标已接受");
        rclcpp::TimerBase::SharedPtr cancel_timer;
        if (cancel_after > 0.0) {
          cancel_timer = node->create_wall_timer(std::chrono::duration<double>(cancel_after), [&]() {
            cancel_timer->cancel();
            RCLCPP_INFO(node->get_logger(), "发送取消请求");
            client->async_cancel_goal(handle);
          });
        }
        auto future = client->async_get_result(handle);
        if (rclcpp::spin_until_future_complete(node, future, 130s) == rclcpp::FutureReturnCode::SUCCESS) {
          const auto response = future.get();
          RCLCPP_INFO(node->get_logger(), "最终状态码=%d，success=%s", static_cast<int>(response.code),
            response.result->success ? "true" : "false");
          exit_code = (response.code == rclcpp_action::ResultCode::SUCCEEDED || (cancel_after > 0.0 && response.code == rclcpp_action::ResultCode::CANCELED)) ? 0 : 1;
        } else {
          RCLCPP_ERROR(node->get_logger(), "等待结果失败，请求取消目标");
          auto canceled = client->async_cancel_goal(handle);
          rclcpp::spin_until_future_complete(node, canceled, 2s);
        }
        if (cancel_timer) {cancel_timer->cancel();}
      }
    }
  } else {RCLCPP_ERROR(node->get_logger(), "10 秒内没有找到 Action Server");}
  rclcpp::shutdown();
  return exit_code;
}
