#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class LoggerDemo : public rclcpp::Node
{
public:
  LoggerDemo()
  : Node("logger_demo"), count_(0)
  {
    timer_ = create_wall_timer(1s, std::bind(&LoggerDemo::tick, this));
    RCLCPP_INFO(get_logger(), "LOGGER_CPP_OK logger demo started");
  }

private:
  void tick()
  {
    ++count_;
    RCLCPP_DEBUG(get_logger(), "DEBUG message #%zu", count_);
    RCLCPP_INFO(get_logger(), "INFO message #%zu", count_);
    RCLCPP_WARN(get_logger(), "WARN message #%zu", count_);
    if (count_ == 3) {
      RCLCPP_ERROR(get_logger(), "ERROR: exception on message 3");
    }
    if (count_ >= 5) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "high-frequency warning (throttled)");
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoggerDemo>());
  rclcpp::shutdown();
  return 0;
}
