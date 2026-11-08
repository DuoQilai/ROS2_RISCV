#pragma once
#include <chrono>
#include <cmath>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace std::chrono_literals;
inline volatile std::sig_atomic_t navigation_stop = 0;

inline void init_navigation(int argc, char ** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) { navigation_stop = 1; });
  std::signal(SIGTERM, [](int) { navigation_stop = 1; });
}

class BTNavigatorClient : public rclcpp::Node {
public:
  using Navigate = nav2_msgs::action::NavigateToPose;
  using Handle = rclcpp_action::ClientGoalHandle<Navigate>;
  BTNavigatorClient() : Node("bt_navigator_client") {
    client_ = rclcpp_action::create_client<Navigate>(this, "navigate_to_pose");
  }
  bool navigate(double x, double y, double yaw, const std::string & tree = "",
                double cancel_after = 0) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
        !std::isfinite(cancel_after) || cancel_after < 0) {
      RCLCPP_ERROR(get_logger(), "目标必须为有限数值，取消延时不能为负");
      return false;
    }
    if (!client_->wait_for_action_server(15s) || navigation_stop) {
      RCLCPP_ERROR(get_logger(), "navigate_to_pose 服务未就绪");
      return false;
    }
    Navigate::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.z = std::sin(yaw / 2);
    goal.pose.pose.orientation.w = std::cos(yaw / 2);
    goal.behavior_tree = tree;
    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    options.feedback_callback = [this](Handle::SharedPtr, const auto feedback) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "剩余距离: %.2f m",
        feedback->distance_remaining);
    };
    auto sent = client_->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(shared_from_this(), sent, 10s) !=
        rclcpp::FutureReturnCode::SUCCESS || !sent.get()) {
      RCLCPP_ERROR(get_logger(), "目标未接受");
      return false;
    }
    auto handle = sent.get();
    RCLCPP_INFO(get_logger(), "目标已接受: x=%.2f y=%.2f yaw=%.2f", x, y, yaw);
    auto result = client_->async_get_result(handle);
    const auto start = std::chrono::steady_clock::now();
    bool canceled = false;
    double cancel_time = 0;
    while (rclcpp::ok() && result.wait_for(0s) != std::future_status::ready) {
      rclcpp::spin_some(shared_from_this());
      double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (!canceled && (navigation_stop || (cancel_after > 0 && elapsed >= cancel_after) || elapsed >= 180)) {
        client_->async_cancel_goal(handle);
        canceled = true;
        cancel_time = elapsed;
        RCLCPP_INFO(get_logger(), "已请求取消导航");
      }
      if (canceled && elapsed - cancel_time >= 10) {
        RCLCPP_ERROR(get_logger(), "取消后未收到结果");
        return false;
      }
      rclcpp::sleep_for(20ms);
    }
    if (!rclcpp::ok()) { return false; }
    auto code = result.get().code;
    RCLCPP_INFO(get_logger(), "导航结果: %s",
      code == rclcpp_action::ResultCode::SUCCEEDED ? "SUCCEEDED" :
      code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" : "ABORTED");
    return code == rclcpp_action::ResultCode::SUCCEEDED;
  }
private:
  rclcpp_action::Client<Navigate>::SharedPtr client_;
};
