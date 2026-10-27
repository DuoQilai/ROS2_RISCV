#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <stdexcept>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}

class PatrolDriver : public rclcpp::Node
{
public:
  PatrolDriver() : Node("patrol_driver")
  {
    linear_ = declare_parameter("linear_speed", 0.12);
    angular_ = declare_parameter("angular_speed", 0.45);
    loop_ = declare_parameter("loop", true);
    duration_ = declare_parameter("duration", 0.0);
    if (!std::isfinite(linear_) || !std::isfinite(angular_) || !std::isfinite(duration_) || duration_ < 0.0) {
      throw std::invalid_argument("巡航参数必须为有限数值，duration 不能为负");
    }
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    started_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(50ms, [this]() {
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
      if (duration_ > 0.0 && elapsed >= duration_ && !loop_) {done_ = true; stop(); return;}
      const double phase = std::fmod(elapsed, 31.4);
      geometry_msgs::msg::Twist command;
      if (phase < 5.0 || (phase >= 7.85 && phase < 12.85) ||
        (phase >= 15.70 && phase < 20.70) || (phase >= 23.55 && phase < 28.55))
      {command.linear.x = linear_;}
      else {command.angular.z = angular_;}
      publisher_->publish(command);
    });
    RCLCPP_INFO(get_logger(), "Patrol driver ready: speed=%.2f m/s, turn=%.2f rad/s", linear_, angular_);
  }
  bool done() const {return done_;}
  void stop() {timer_->cancel(); publisher_->publish(geometry_msgs::msg::Twist());}
private:
  double linear_, angular_, duration_;
  bool loop_, done_{false};
  std::chrono::steady_clock::time_point started_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  int status = 0;
  try {
    auto node = std::make_shared<PatrolDriver>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (!stop_requested && rclcpp::ok() && !node->done()) {executor.spin_once(50ms);}
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {node->stop(); std::this_thread::sleep_for(100ms);}
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("patrol_driver"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
