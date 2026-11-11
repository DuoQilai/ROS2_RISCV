#pragma once
#include <chrono>
#include <cmath>
#include <csignal>
#include <type_traits>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;
inline volatile std::sig_atomic_t stop_navigation = 0;
inline void init_navigation(int argc, char ** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) { stop_navigation = 1; });
  std::signal(SIGTERM, [](int) { stop_navigation = 1; });
}

class Navigator : public rclcpp::Node {
public:
  using Navigate = nav2_msgs::action::NavigateToPose;
  using Follow = nav2_msgs::action::FollowWaypoints;
  Navigator() : Node("navigation_lab") {}

  geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = "map";
    result.header.stamp = now();
    result.pose.position.x = x;
    result.pose.position.y = y;
    result.pose.orientation.z = std::sin(yaw / 2);
    result.pose.orientation.w = std::cos(yaw / 2);
    return result;
  }

  bool ready() {
    using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
    bool localized = false;
    auto sub = create_subscription<Pose>("amcl_pose", 10,
      [&localized](Pose::ConstSharedPtr) { localized = true; });
    auto pub = create_publisher<Pose>("initialpose", 10);
    auto state = create_client<lifecycle_msgs::srv::GetState>("bt_navigator/get_state");
    for (int attempt = 0; attempt < 30 && rclcpp::ok() && !stop_navigation; ++attempt) {
      if (!localized) {
        Pose initial;
        initial.header = pose(0, 0, 0).header;
        initial.pose.pose.orientation.w = 1;
        pub->publish(initial);
      }
      if (state->wait_for_service(1s)) {
        auto request = state->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
        if (rclcpp::spin_until_future_complete(shared_from_this(), request, 1s) ==
            rclcpp::FutureReturnCode::SUCCESS && request.get()->current_state.id == 3 && localized) {
          RCLCPP_INFO(get_logger(), "Nav2 active; initial pose=(0,0,0)");
          return true;
        }
      }
      rclcpp::spin_some(shared_from_this());
      rclcpp::sleep_for(1s);
    }
    RCLCPP_ERROR(get_logger(), "Nav2 or initial localization is not ready");
    return false;
  }

  bool navigate(double x, double y, double yaw) {
    Navigate::Goal goal;
    goal.pose = pose(x, y, yaw);
    return run<Navigate>("navigate_to_pose", goal);
  }

  bool follow_waypoints() {
    const double pi = std::acos(-1.0);
    Follow::Goal goal;
    goal.poses = {pose(2, 0, 0), pose(2, 2, pi / 2), pose(0, 2, pi), pose(0, 0, -pi / 2)};
    return run<Follow>("follow_waypoints", goal);
  }

private:
  // Only the two actions used by the original lab share this send/wait/cancel loop.
  template<class Action>
  bool run(const std::string & name, const typename Action::Goal & goal) {
    auto client = rclcpp_action::create_client<Action>(this, name);
    if (!client->wait_for_action_server(15s) || stop_navigation) {
      RCLCPP_ERROR(get_logger(), "%s is not ready", name.c_str());
      return false;
    }
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.feedback_callback = [this](auto, const auto feedback) {
      if constexpr (std::is_same_v<Action, Navigate>) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "distance_remaining=%.2fm navigation_time=%ds estimated_time_remaining=%ds number_of_recoveries=%d",
          feedback->distance_remaining, feedback->navigation_time.sec,
          feedback->estimated_time_remaining.sec, feedback->number_of_recoveries);
      } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "current_waypoint=%u (waypoint %u/4)", feedback->current_waypoint, feedback->current_waypoint + 1);
      }
    };
    auto sent = client->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(shared_from_this(), sent, 10s) !=
        rclcpp::FutureReturnCode::SUCCESS || !sent.get()) {
      RCLCPP_ERROR(get_logger(), "%s goal was not accepted", name.c_str());
      return false;
    }
    auto result = client->async_get_result(sent.get());
    bool canceled = false;
    auto cancel_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && result.wait_for(0s) != std::future_status::ready) {
      rclcpp::spin_some(shared_from_this());
      if (stop_navigation && !canceled) {
        client->async_cancel_goal(sent.get());
        canceled = true;
        cancel_time = std::chrono::steady_clock::now();
      }
      if (canceled && std::chrono::steady_clock::now() - cancel_time > 10s) {
        RCLCPP_ERROR(get_logger(), "No cancellation result; check robot stop feedback");
        return false;
      }
      rclcpp::sleep_for(20ms);
    }
    if (!rclcpp::ok()) { return false; }
    auto response = result.get();
    const bool succeeded = response.code == rclcpp_action::ResultCode::SUCCEEDED;
    RCLCPP_INFO(get_logger(), "%s: %s", name.c_str(), succeeded ? "SUCCEEDED" :
      response.code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" : "FAILED");
    if constexpr (std::is_same_v<Action, Follow>) {
      for (auto index : response.result->missed_waypoints) {
        RCLCPP_WARN(get_logger(), "missed_waypoint=%d", index);
      }
      return succeeded && response.result->missed_waypoints.empty();
    }
    return succeeded;
  }
};
