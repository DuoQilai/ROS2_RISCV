#include <chrono>
#include <csignal>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "dishes_action_interfaces/action/do_dishes.hpp"

using namespace std::chrono_literals;
namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}

class DishesServer : public rclcpp::Node
{
public:
  using Action = dishes_action_interfaces::action::DoDishes;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  DishesServer() : Node("dishes_action_server")
  {
    server_ = rclcpp_action::create_server<Action>(this, "do_dishes_lab",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        if (busy_ || goal->total_dishes == 0) {
          RCLCPP_WARN(get_logger(), "拒绝目标：已有任务或盘子数量为零");
          return rclcpp_action::GoalResponse::REJECT;
        }
        busy_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<Handle>) {
        RCLCPP_INFO(get_logger(), "收到取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<Handle> handle) {
        goal_ = handle;
        cleaned_ = 0;
        next_ = std::chrono::steady_clock::now() + 1s;
        RCLCPP_INFO(get_logger(), "接受目标：%u", goal_->get_goal()->total_dishes);
      });
    timer_ = create_wall_timer(100ms, [this]() {tick();});
  }

  void stop()
  {
    timer_->cancel();
    if (goal_) {finish(false);}
  }

private:
  void finish(bool success)
  {
    auto result = std::make_shared<Action::Result>();
    result->cleaned_dishes = cleaned_;
    result->success = success && !goal_->is_canceling();
    if (goal_->is_canceling()) {goal_->canceled(result);}
    else if (success) {goal_->succeed(result);}
    else {goal_->abort(result);}
    RCLCPP_INFO(get_logger(), "任务结束：cleaned=%u success=%s", cleaned_, success ? "true" : "false");
    goal_.reset();
    busy_ = false;
  }

  void tick()
  {
    if (!goal_) {return;}
    if (stop_requested || goal_->is_canceling()) {finish(false); return;}
    if (std::chrono::steady_clock::now() < next_) {return;}
    if (cleaned_ == goal_->get_goal()->total_dishes) {finish(true); return;}
    ++cleaned_;
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->progress = static_cast<float>(cleaned_) / goal_->get_goal()->total_dishes;
    feedback->current_dish = cleaned_;
    goal_->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "进度: %.0f%%", feedback->progress * 100.0);
    next_ += 1s;
    if (cleaned_ == goal_->get_goal()->total_dishes) {finish(!goal_->is_canceling());}
  }

  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Handle> goal_;
  bool busy_{false};
  uint32_t cleaned_{0};
  std::chrono::steady_clock::time_point next_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  auto node = std::make_shared<DishesServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (!stop_requested && rclcpp::ok()) {executor.spin_once(100ms);}
  node->stop();
  std::this_thread::sleep_for(100ms);
  rclcpp::shutdown();
  return 0;
}
