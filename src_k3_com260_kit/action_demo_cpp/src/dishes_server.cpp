#include <chrono>
#include <csignal>
#include <memory>
#include <limits>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_demo_interfaces/action/do_dishes.hpp"

using namespace std::chrono_literals;
namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}

class DishesServer : public rclcpp::Node
{
public:
  using Action = action_demo_interfaces::action::DoDishes;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  DishesServer() : Node("dishes_server")
  {
    server_ = rclcpp_action::create_server<Action>(this, "dishes",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        if (busy_ || goal->dishwasher_id == 0 ||
          goal->dishwasher_id > std::numeric_limits<uint32_t>::max() / 5) {
          RCLCPP_WARN(get_logger(), "拒绝目标：已有任务、洗碗机编号为零或结果超出 uint32 范围");
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
        next_ = std::chrono::steady_clock::now() + 0ms;
        RCLCPP_INFO(get_logger(), "接受目标：%u", goal_->get_goal()->dishwasher_id);
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
    result->total_dishes_cleaned = cleaned_ * goal_->get_goal()->dishwasher_id;
    if (goal_->is_canceling()) {goal_->canceled(result);}
    else if (success) {goal_->succeed(result);}
    else {goal_->abort(result);}
    RCLCPP_INFO(get_logger(), "任务结束：cleaned=%u success=%s", cleaned_ * goal_->get_goal()->dishwasher_id, success ? "true" : "false");
    goal_.reset();
    busy_ = false;
  }

  void tick()
  {
    if (!goal_) {return;}
    if (stop_requested || goal_->is_canceling()) {finish(false); return;}
    if (std::chrono::steady_clock::now() < next_) {return;}
    if (cleaned_ == 5) {finish(true); return;}
    ++cleaned_;
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->percent_complete = cleaned_ * 20.0F;
    goal_->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "进度: %.0f%%", static_cast<double>(feedback->percent_complete));
    next_ += 500ms;

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
