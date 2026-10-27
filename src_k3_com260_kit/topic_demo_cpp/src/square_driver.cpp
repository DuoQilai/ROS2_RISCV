#include <algorithm>
#include <chrono>
#include <csignal>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}

class SquareDriver : public rclcpp::Node
{
public:
  SquareDriver()
  : Node("square_driver")
  {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    motion_clock_ = get_parameter("use_sim_time").as_bool() ?
      get_clock() : std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    RCLCPP_INFO(get_logger(), "SQUARE_DRIVER_ACTIVE waiting=5s");
  }

  bool drive_square()
  {
    for (int i = 0; i < 50 && !stop_requested && rclcpp::ok(); ++i) {
      std::this_thread::sleep_for(100ms);
    }
    if (stop_requested || !rclcpp::ok()) {return false;}
    if (get_parameter("use_sim_time").as_bool() && now().nanoseconds() == 0) {
      RCLCPP_ERROR(get_logger(), "No simulation clock received; motion not started");
      return false;
    }
    bool completed = true;
    for (int side = 1; side <= 4; ++side) {
      RCLCPP_INFO(get_logger(), "SQUARE_SIDE %d straight", side);
      if (!move(0.2, 0.0, 5s)) {completed = false; break;}
      RCLCPP_INFO(get_logger(), "SQUARE_SIDE %d turn", side);
      if (!move(0.0, 1.57, 1s)) {completed = false; break;}
    }
    // Stop using wall time so a paused simulation cannot block shutdown.
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
      publisher_->publish(geometry_msgs::msg::Twist());
      std::this_thread::sleep_for(100ms);
    }
    completed = completed && !stop_requested && rclcpp::ok();
    if (completed) {RCLCPP_INFO(get_logger(), "SQUARE_DRIVER_DONE");}
    else {RCLCPP_WARN(get_logger(), "SQUARE_DRIVER_INTERRUPTED");}
    return completed;
  }

private:
  bool move(double linear, double angular, std::chrono::milliseconds duration)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = linear;
    message.angular.z = angular;
    const auto end = motion_clock_->now() + rclcpp::Duration(duration);
    while (!stop_requested && rclcpp::ok() && motion_clock_->now() < end) {
      publisher_->publish(message);
      const auto next = std::min(end, motion_clock_->now() + rclcpp::Duration(100ms));
      while (!stop_requested && rclcpp::ok() && motion_clock_->now() < next) {
        std::this_thread::sleep_for(10ms);
      }
    }
    return !stop_requested && rclcpp::ok();
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr motion_clock_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  auto node = std::make_shared<SquareDriver>();
  std::thread executor([node]() {rclcpp::spin(node);});
  const bool completed = node->drive_square();
  rclcpp::shutdown();
  executor.join();
  return completed ? 0 : 1;
}
