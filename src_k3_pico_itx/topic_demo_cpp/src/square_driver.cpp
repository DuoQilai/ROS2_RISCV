#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

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
    std::this_thread::sleep_for(5s);
    if (get_parameter("use_sim_time").as_bool() && now().nanoseconds() == 0) {
      RCLCPP_ERROR(get_logger(), "No simulation clock received; motion not started");
      return false;
    }
    for (int side = 1; side <= 4 && rclcpp::ok(); ++side) {
      RCLCPP_INFO(get_logger(), "SQUARE_SIDE %d straight", side);
      move(0.2, 0.0, 5s);
      RCLCPP_INFO(get_logger(), "SQUARE_SIDE %d turn", side);
      move(0.0, 1.57, 1s);
    }
    move(0.0, 0.0, 500ms);
    RCLCPP_INFO(get_logger(), "SQUARE_DRIVER_DONE");
    return true;
  }

private:
  void move(double linear, double angular, std::chrono::milliseconds duration)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = linear;
    message.angular.z = angular;
    const auto end = motion_clock_->now() + rclcpp::Duration(duration);
    while (rclcpp::ok() && motion_clock_->now() < end) {
      publisher_->publish(message);
      if (!motion_clock_->sleep_until(
          std::min(end, motion_clock_->now() + rclcpp::Duration(100ms)))) {
        break;
      }
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr motion_clock_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SquareDriver>();
  std::thread executor([node]() {rclcpp::spin(node);});
  const bool completed = node->drive_square();
  rclcpp::shutdown();
  executor.join();
  return completed ? 0 : 1;
}
