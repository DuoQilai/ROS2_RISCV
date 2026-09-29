#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;
namespace
{
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}

class SpeedController : public rclcpp::Node
{
public:
  SpeedController() : Node("speed_controller")
  {
    declare_parameter("linear_speed", 0.2);
    declare_parameter("angular_speed", 0.0);
    declare_parameter("enable_control", true);
    const auto initial = validate(get_parameters({"linear_speed", "angular_speed"}));
    if (!initial.successful) {throw std::invalid_argument(initial.reason);}
    callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return validate(params);});
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = create_wall_timer(100ms, [this]() {
      geometry_msgs::msg::Twist command;
      const bool enabled = get_parameter("enable_control").as_bool();
      if (enabled) {
        command.linear.x = get_parameter("linear_speed").as_double();
        command.angular.z = get_parameter("angular_speed").as_double();
      }
      publisher_->publish(command);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "enable=%s, v=%.2f m/s, w=%.2f rad/s", enabled ? "true" : "false",
        command.linear.x, command.angular.z);
    });
  }

  void stop()
  {
    timer_->cancel();
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
      publisher_->publish(geometry_msgs::msg::Twist());
      std::this_thread::sleep_for(100ms);
    }
  }

private:
  rcl_interfaces::msg::SetParametersResult validate(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : params) {
      const auto & name = param.get_name();
      if (name != "linear_speed" && name != "angular_speed") {continue;}
      const double limit = name == "linear_speed" ? 1.0 : 2.0;
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
        !std::isfinite(param.as_double()) || std::abs(param.as_double()) > limit)
      {
        result.successful = false;
        result.reason = name == "linear_speed" ?
          "线速度必须是 [-1.0, 1.0] m/s 范围内的有限浮点数" :
          "角速度必须是 [-2.0, 2.0] rad/s 范围内的有限浮点数";
        break;
      }
    }
    return result;
  }
  OnSetParametersCallbackHandle::SharedPtr callback_;
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
    auto node = std::make_shared<SpeedController>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (!stop_requested && rclcpp::ok()) {executor.spin_once(100ms);}
    node->stop();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("speed_controller"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
