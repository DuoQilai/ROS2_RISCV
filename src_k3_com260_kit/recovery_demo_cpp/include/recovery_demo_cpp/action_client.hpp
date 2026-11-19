#pragma once
#include <chrono>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;
inline volatile std::sig_atomic_t stop_requested = 0;
inline void init_actions(int argc, char ** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) { stop_requested = 1; });
  std::signal(SIGTERM, [](int) { stop_requested = 1; });
}

// The recovery and behavior-tree examples share the same result/cancel handling.
template<class Action>
bool run_action(const rclcpp::Node::SharedPtr & node, const std::string & name,
                const typename Action::Goal & goal, double wall_timeout = 0) {
  auto client = rclcpp_action::create_client<Action>(node, name);
  if (stop_requested || !client->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node->get_logger(), "%s 未就绪", name.c_str());
    return false;
  }
  auto sent = client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, sent, 10s) !=
      rclcpp::FutureReturnCode::SUCCESS || !sent.get()) {
    RCLCPP_ERROR(node->get_logger(), "%s 目标未接受", name.c_str());
    return false;
  }
  auto result = client->async_get_result(sent.get());
  auto start = std::chrono::steady_clock::now();
  bool canceled = false;
  auto cancel_time = start;
  while (rclcpp::ok() && result.wait_for(0s) != std::future_status::ready) {
    rclcpp::spin_some(node);
    auto now = std::chrono::steady_clock::now();
    const bool timed_out = wall_timeout > 0 && now - start >= std::chrono::duration<double>(wall_timeout);
    if (!canceled && (stop_requested || timed_out)) {
      client->async_cancel_goal(sent.get());
      canceled = true;
      cancel_time = now;
      RCLCPP_INFO(node->get_logger(), "%s 已请求取消", name.c_str());
    }
    if (canceled && now - cancel_time > 10s) {
      RCLCPP_ERROR(node->get_logger(), "%s 取消结果超时，请检查停车反馈", name.c_str());
      return false;
    }
    rclcpp::sleep_for(20ms);
  }
  if (!rclcpp::ok()) { return false; }
  const auto code = result.get().code;
  RCLCPP_INFO(node->get_logger(), "%s: %s", name.c_str(),
    code == rclcpp_action::ResultCode::SUCCEEDED ? "SUCCEEDED" :
    code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" : "ABORTED");
  return code == rclcpp_action::ResultCode::SUCCEEDED;
}
