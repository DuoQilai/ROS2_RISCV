#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tracking_interfaces/action/tracking.hpp"

using namespace std::chrono_literals;
using Action = tracking_interfaces::action::Tracking;
int main(int argc, char ** argv)
{
  double x = 0.0, y = 0.0, third = 0.0;
  try {
    auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() < 3 || args.size() > 4) {throw std::invalid_argument("用法：ros2 run tracking_server client x y [z]");}
    for (size_t i = 1; i < args.size(); ++i) {
      size_t used = 0;
      double value = std::stod(args[i], &used);
      if (used != args[i].size() || !std::isfinite(value)) {throw std::invalid_argument("坐标必须为有限数字");}
      if (i == 1) {x = value;} else if (i == 2) {y = value;}
      else if (i == 3) {third = value;}
    }

  } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 2;}
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tracking_client");
  auto client = rclcpp_action::create_client<Action>(node, "tracking");
  int exit_code = 1;
  if (client->wait_for_action_server(10s)) {
    Action::Goal goal;
    goal.target.x = x;
    goal.target.y = y;
    goal.target.z = third;
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.feedback_callback = [node](auto, const auto feedback) {
      RCLCPP_INFO(node->get_logger(), "current_position=%.6f m distance=%.6f m", feedback->current_position, feedback->distance);
    };
    auto sent = client->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(node, sent, 10s) == rclcpp::FutureReturnCode::SUCCESS) {
      auto handle = sent.get();
      if (!handle) {RCLCPP_WARN(node->get_logger(), "目标被 Server 拒绝");}
      else {
        RCLCPP_INFO(node->get_logger(), "目标已接受");
        rclcpp::TimerBase::SharedPtr cancel_timer;

        auto future = client->async_get_result(handle);
        if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
          const auto response = future.get();
          RCLCPP_INFO(node->get_logger(), "最终状态码=%d，success=%s", static_cast<int>(response.code),
            response.result->success ? "true" : "false");
          exit_code = (response.code == rclcpp_action::ResultCode::SUCCEEDED) ? 0 : 1;
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
