#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "speed_interfaces/srv/speed_control.hpp"

using namespace std::chrono_literals;

bool parse_number(const char * text, double & value)
{
  errno = 0;
  char * end = nullptr;
  value = std::strtod(text, &end);
  return errno != ERANGE && end != text && *end == '\0' && std::isfinite(value);
}

class SpeedClient : public rclcpp::Node
{
public:
  SpeedClient()
  : Node("speed_client")
  {
    client_ = create_client<speed_interfaces::srv::SpeedControl>("speed_control");
  }

  int run(double linear, double angular, double duration)
  {
    if (!client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "SPEED_SERVICE_UNAVAILABLE");
      return 1;
    }
    auto request = std::make_shared<speed_interfaces::srv::SpeedControl::Request>();
    request->linear_x = linear;
    request->angular_z = angular;
    request->duration = duration;
    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
        shared_from_this(), future,
        std::chrono::duration<double>(std::max(0.0, duration) + 5.0)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return 1;
    }
    const auto response = future.get();
    RCLCPP_INFO(get_logger(), "SPEED_RESULT success=%s message=%s", response->success ? "true" : "false", response->message.c_str());
    return response->success ? 0 : 1;
  }

private:
  rclcpp::Client<speed_interfaces::srv::SpeedControl>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  if (argc > 4) {
    std::cerr << "Usage: ros2 run speed_control speed_client [linear] [angular] [duration]\n";
    return 2;
  }
  double linear = 0.2;
  double angular = 0.0;
  double duration = 3.0;
  if ((argc > 1 && !parse_number(argv[1], linear)) ||
    (argc > 2 && !parse_number(argv[2], angular)) ||
    (argc > 3 && !parse_number(argv[3], duration)))
  {
    std::cerr << "linear, angular and duration must be finite numbers\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpeedClient>();
  const int result = node->run(linear, angular, duration);
  rclcpp::shutdown();
  return result;
}
