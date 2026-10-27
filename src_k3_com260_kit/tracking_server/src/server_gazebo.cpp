#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <thread>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tracking_interfaces/action/tracking.hpp"

using namespace std::chrono_literals;
namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
double normalize(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}
bool valid_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
    std::isfinite(pose.position.z) && std::isfinite(norm) && norm >= 1e-9;
}
double yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
  return std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}
}

class NavigationServer : public rclcpp::Node
{
public:
  using Action = tracking_interfaces::action::Tracking;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  NavigationServer() : Node("tracking_server_gazebo")
  {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        if (message->header.frame_id != "odom" || !valid_pose(message->pose.pose)) {return;}
        if (goal_ && received_) {
          traveled_ += std::hypot(message->pose.pose.position.x - pose_.position.x,
            message->pose.pose.position.y - pose_.position.y);
        }
        pose_ = message->pose.pose;
        received_ = true;
        last_odom_ = std::chrono::steady_clock::now();
      });
    server_ = rclcpp_action::create_server<Action>(this, "tracking",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        const bool valid = std::isfinite(goal->target.x) && std::isfinite(goal->target.y) && goal->target.z == 0.0;
        if (!valid || busy_ || !received_ || std::chrono::steady_clock::now() - last_odom_ > 1s) {
          RCLCPP_WARN(get_logger(), "拒绝目标：坐标无效、已有任务或 /odom 不可用");
          return rclcpp_action::GoalResponse::REJECT;
        }
        busy_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<Handle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<Handle> handle) {
        goal_ = handle;
        started_ = std::chrono::steady_clock::now();
        position_reached_ = false;
        traveled_ = 0.0;
        RCLCPP_INFO(get_logger(), "接受导航目标");
      });
    timer_ = create_wall_timer(100ms, [this]() {tick();});
  }
  void stop()
  {
    timer_->cancel();
    if (goal_) {finish(false);}
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
      publisher_->publish(geometry_msgs::msg::Twist());
      std::this_thread::sleep_for(100ms);
    }
  }

private:
  void finish(bool success)
  {
    publisher_->publish(geometry_msgs::msg::Twist());
    auto result = std::make_shared<Action::Result>();
    result->success = success && !goal_->is_canceling();
    if (goal_->is_canceling()) {goal_->canceled(result);}
    else if (result->success) {goal_->succeed(result);}
    else {goal_->abort(result);}
    RCLCPP_INFO(get_logger(), "导航结束：success=%s x=%.3f y=%.3f yaw=%.3f",
      result->success ? "true" : "false", pose_.position.x, pose_.position.y, yaw(pose_.orientation));
    goal_.reset();
    busy_ = false;
  }
  void tick()
  {
    if (!goal_) {return;}
    const auto now = std::chrono::steady_clock::now();
    if (stop_requested || goal_->is_canceling()) {finish(false); return;}
    if (now - last_odom_ > 1s || now - started_ > 120s) {
      RCLCPP_ERROR(get_logger(), "导航超时或 /odom 已过期，停止机器人");
      finish(false); return;
    }
    const auto & target = goal_->get_goal()->target;
    const double dx = target.x - pose_.position.x, dy = target.y - pose_.position.y;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance)) {finish(false); return;}
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->current_position = traveled_;
    feedback->distance = distance;
    goal_->publish_feedback(feedback);
    geometry_msgs::msg::Twist command;
    const double current_yaw = yaw(pose_.orientation);
    if (!position_reached_) {
      if (distance <= 0.10) {
        position_reached_ = true;
        publisher_->publish(command);
        finish(!goal_->is_canceling());
        return;
      }
      const double error = normalize(std::atan2(dy, dx) - current_yaw);
      command.angular.z = std::clamp(1.8 * error, -1.0, 1.0);
      if (std::abs(error) <= 20.0 * std::acos(-1.0) / 180.0) {
        command.linear.x = std::min(0.25, 0.8 * distance);
      }
    }
    publisher_->publish(command);
  }
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Handle> goal_;
  geometry_msgs::msg::Pose pose_;
  std::chrono::steady_clock::time_point last_odom_, started_;
  bool busy_{false}, received_{false}, position_reached_{false};
  double traveled_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  auto node = std::make_shared<NavigationServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (!stop_requested && rclcpp::ok()) {executor.spin_once(100ms);}
  node->stop();
  rclcpp::shutdown();
  return 0;
}
