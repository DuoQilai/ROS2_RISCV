#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "recovery_demo_cpp/action_client.hpp"

class Navigate : public BT::StatefulActionNode {
  using Action = nav2_msgs::action::NavigateToPose;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
public:
  Navigate(const std::string & name, const BT::NodeConfiguration & config,
           rclcpp::Node::SharedPtr node, double x, double y)
  : StatefulActionNode(name, config), node_(node),
    client_(rclcpp_action::create_client<Action>(node, "navigate_to_pose")) {
    goal_.pose.header.frame_id = "map";
    goal_.pose.pose.position.x = x;
    goal_.pose.pose.position.y = y;
    goal_.pose.pose.orientation.w = 1;
  }
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    if (stop_requested || !client_->wait_for_action_server(10s)) { return BT::NodeStatus::FAILURE; }
    handle_.reset();
    result_ = {};
    goal_.pose.header.stamp = node_->now();
    sent_ = client_->async_send_goal(goal_);
    start_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if (std::chrono::steady_clock::now() - start_ > 180s) {
      onHalted();
      return BT::NodeStatus::FAILURE;
    }
    if (!handle_) {
      if (sent_.wait_for(0s) != std::future_status::ready) { return BT::NodeStatus::RUNNING; }
      handle_ = sent_.get();
      if (!handle_) { return BT::NodeStatus::FAILURE; }
      result_ = client_->async_get_result(handle_);
    }
    if (result_.wait_for(0s) != std::future_status::ready) { return BT::NodeStatus::RUNNING; }
    return result_.get().code == rclcpp_action::ResultCode::SUCCEEDED
      ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  void onHalted() override {
    if (!handle_ && sent_.valid() && rclcpp::spin_until_future_complete(node_, sent_, 10s) ==
        rclcpp::FutureReturnCode::SUCCESS) { handle_ = sent_.get(); }
    if (!handle_) { return; }
    if (!result_.valid()) { result_ = client_->async_get_result(handle_); }
    if (result_.wait_for(0s) == std::future_status::ready) { return; }
    client_->async_cancel_goal(handle_);
    if (rclcpp::spin_until_future_complete(node_, result_, 10s) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "导航取消结果超时，请检查停车反馈");
    } else {
      RCLCPP_INFO(node_->get_logger(), "导航停止结果: %s",
        result_.get().code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" : "FINISHED");
    }
  }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  Action::Goal goal_;
  Handle::SharedPtr handle_;
  std::shared_future<Handle::SharedPtr> sent_;
  std::shared_future<Handle::WrappedResult> result_;
  std::chrono::steady_clock::time_point start_;
};
