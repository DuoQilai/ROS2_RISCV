#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_demo_lab_interfaces/action/do_dishes.hpp"

using namespace std::chrono_literals;
using Action = action_demo_lab_interfaces::action::DoDishes;
int main(int argc, char ** argv)
{
  uint32_t total = 10;
  try {
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() > 2) {throw std::invalid_argument("参数过多");}
    if (args.size() == 2) {
      size_t used = 0;
      const auto value = std::stoull(args[1], &used);
      if (args[1].empty() || args[1][0] == '-' || used != args[1].size() ||
        value == 0 || value > std::numeric_limits<uint32_t>::max())
      {throw std::invalid_argument("盘子数量必须为正 uint32 整数");}
      total = static_cast<uint32_t>(value);
    }
  } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 2;}
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("do_dishes_client");
  auto client = rclcpp_action::create_client<Action>(node, "do_dishes");
  int exit_code = 1;
  if (client->wait_for_action_server(10s)) {
    Action::Goal goal;
    goal.total_dishes = total;
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.feedback_callback = [node](auto, const auto feedback) {
      RCLCPP_INFO(node->get_logger(), "收到反馈：进度=%.0f%%，当前盘子=%u", feedback->progress * 100.0, feedback->current_dish);
    };
    auto sent = client->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(node, sent, 10s) == rclcpp::FutureReturnCode::SUCCESS) {
      auto handle = sent.get();
      if (!handle) {RCLCPP_WARN(node->get_logger(), "目标被 Server 拒绝");}
      else {
        RCLCPP_INFO(node->get_logger(), "目标已接受");
        rclcpp::TimerBase::SharedPtr cancel_timer;

        auto result = client->async_get_result(handle);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
          const auto response = result.get();
          RCLCPP_INFO(node->get_logger(), "最终状态码=%d，已清洗=%u，成功=%s",
            static_cast<int>(response.code), response.result->cleaned_dishes, response.result->success ? "true" : "false");
          exit_code = response.code == rclcpp_action::ResultCode::SUCCEEDED ? 0 : 1;
        }
        if (cancel_timer) {cancel_timer->cancel();}
      }
    }
  } else {RCLCPP_ERROR(node->get_logger(), "没有找到 Action Server");}
  rclcpp::shutdown();
  return exit_code;
}
