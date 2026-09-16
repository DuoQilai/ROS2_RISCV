#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "speed_interfaces/srv/speed_control.hpp"

using namespace std::chrono_literals;

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
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
      publisher_->publish(command);
      std::this_thread::sleep_for(100ms);
    }
    publisher_->publish(geometry_msgs::msg::Twist());
    response->success = true;
    response->message = "motion completed";
    RCLCPP_INFO(get_logger(), "SPEED_RESULT success=true");
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Service<speed_interfaces::srv::SpeedControl>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedServer>());
  rclcpp::shutdown();
  return 0;
}
