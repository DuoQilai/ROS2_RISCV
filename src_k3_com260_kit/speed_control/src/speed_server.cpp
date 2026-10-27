#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "speed_interfaces/srv/speed_control.hpp"

using namespace std::chrono_literals;

namespace
{
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}

class SpeedServer : public rclcpp::Node
{
public:
  SpeedServer()
  : Node("speed_server")
  {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    service_ = create_service<speed_interfaces::srv::SpeedControl>(
      "speed_control",
      std::bind(&SpeedServer::handle, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "SPEED_SERVER_READY");
  }

  bool motion_interrupted() const {return motion_interrupted_;}

private:
  void handle(
    const std::shared_ptr<speed_interfaces::srv::SpeedControl::Request> request,
    std::shared_ptr<speed_interfaces::srv::SpeedControl::Response> response)
  {
    if (!std::isfinite(request->duration) || !std::isfinite(request->linear_x) ||
      !std::isfinite(request->angular_z) || request->duration < 0.0 ||
      std::abs(request->linear_x) > 1.0 || std::abs(request->angular_z) > 2.0)
    {
      response->success = false;
      response->message = "speed or duration is outside the allowed range";
      RCLCPP_WARN(get_logger(), "SPEED_REJECTED");
      return;
    }
    geometry_msgs::msg::Twist command;
    command.linear.x = request->linear_x;
    command.angular.z = request->angular_z;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(request->duration);
    while (!stop_requested && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      publisher_->publish(command);
      std::this_thread::sleep_for(100ms);
    }
    publisher_->publish(geometry_msgs::msg::Twist());
    motion_interrupted_ = stop_requested || !rclcpp::ok();
    response->success = !motion_interrupted_;
    response->message = motion_interrupted_ ? "motion interrupted" : "motion completed";
    RCLCPP_INFO(get_logger(), "SPEED_RESULT success=%s", response->success ? "true" : "false");
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Service<speed_interfaces::srv::SpeedControl>::SharedPtr service_;
  bool motion_interrupted_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  auto node = std::make_shared<SpeedServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (!stop_requested && rclcpp::ok()) {executor.spin_once(100ms);}
  // Keep the context alive briefly for the final zero command and response.
  std::this_thread::sleep_for(100ms);
  rclcpp::shutdown();
  return node->motion_interrupted() ? 1 : 0;
}
