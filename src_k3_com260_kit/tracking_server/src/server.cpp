#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tracking_interfaces/action/tracking.hpp"

using namespace std::chrono_literals;
namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}

class TrackingServer : public rclcpp::Node
{
public:
  using Action = tracking_interfaces::action::Tracking;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  TrackingServer() : Node("tracking_server")
  {
    server_ = rclcpp_action::create_server<Action>(this, "tracking",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        const auto & p = goal->target;
        if (busy_ || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(std::hypot(p.x, p.y, p.z)))
        {return rclcpp_action::GoalResponse::REJECT;}
        busy_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<Handle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<Handle> handle) {
        goal_ = handle;
        const auto & p = goal_->get_goal()->target;
        total_ = std::hypot(p.x, p.y, p.z);
        position_ = 0.0;
        next_ = std::chrono::steady_clock::now() + 100ms;
        RCLCPP_INFO(get_logger(), "接受目标：distance=%.6f m", total_);
      });
    timer_ = create_wall_timer(100ms, [this]() {tick();});
  }
  void stop() {timer_->cancel(); if (goal_) {finish(false);}}

private:
  void finish(bool success)
  {
    auto result = std::make_shared<Action::Result>();
    result->success = success && !goal_->is_canceling();
    if (goal_->is_canceling()) {goal_->canceled(result);}
    else if (result->success) {goal_->succeed(result);}
    else {goal_->abort(result);}
    RCLCPP_INFO(get_logger(), "Tracking 结束：success=%s", result->success ? "true" : "false");
    goal_.reset();
    busy_ = false;
  }
  void tick()
  {
    if (!goal_) {return;}
    if (stop_requested || goal_->is_canceling()) {finish(false); return;}
    if (std::chrono::steady_clock::now() < next_) {return;}
    position_ = std::min(position_ + 0.25 * 0.1, total_);
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->current_position = position_;
    feedback->distance = total_ - position_;
    goal_->publish_feedback(feedback);
    next_ += 100ms;
    if (position_ >= total_) {finish(!goal_->is_canceling());}
  }
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Handle> goal_;
  std::chrono::steady_clock::time_point next_;
  bool busy_{false};
  double total_{0.0}, position_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  auto node = std::make_shared<TrackingServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (!stop_requested && rclcpp::ok()) {executor.spin_once(100ms);}
  node->stop();
  std::this_thread::sleep_for(100ms);
  rclcpp::shutdown();
  return 0;
}
