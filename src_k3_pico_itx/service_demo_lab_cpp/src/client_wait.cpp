#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

bool parse_integer(const char * text, std::int64_t & value)
{
  errno = 0;
  char * end = nullptr;
  const long long parsed = std::strtoll(text, &end, 10);
  if (errno == ERANGE || end == text || *end != '\0') {
    return false;
  }
  value = static_cast<std::int64_t>(parsed);
  return true;
}

class AddWaitClient : public rclcpp::Node
{
public:
  AddWaitClient()
  : Node("add_two_ints_client")
  {
    client_ = create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  }

  int run(std::int64_t a, std::int64_t b)
  {
    for (int attempt = 1; attempt <= 3; ++attempt) {
      RCLCPP_INFO(get_logger(), "第 %d/3 次检查服务", attempt);
      if (client_->wait_for_service(2s)) {
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        auto future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future, 5s) ==
          rclcpp::FutureReturnCode::SUCCESS)
        {
          try {
            RCLCPP_INFO(get_logger(), "ADD_RESULT sum=%ld", future.get()->sum);
            return 0;
          } catch (const std::exception & error) {
            RCLCPP_ERROR(get_logger(), "服务调用失败：%s", error.what());
          }
        } else {
          client_->remove_pending_request(future);
          RCLCPP_WARN(get_logger(), "等待服务响应超时");
        }
      } else {
        RCLCPP_WARN(get_logger(), "Server 尚未上线");
      }
      if (attempt < 3) {
        RCLCPP_INFO(get_logger(), "2 秒后重试");
        std::this_thread::sleep_for(2s);
      }
    }
    RCLCPP_ERROR(get_logger(), "3 次尝试均失败");
    return 1;
  }

private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  if (argc > 3) {
    std::cerr << "Usage: ros2 run service_demo_lab_cpp client_wait [a] [b]\n";
    return 2;
  }
  std::int64_t a = 5;
  std::int64_t b = 3;
  if ((argc > 1 && !parse_integer(argv[1], a)) ||
    (argc > 2 && !parse_integer(argv[2], b)))
  {
    std::cerr << "a and b must be integers\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddWaitClient>();
  const int result = node->run(a, b);
  rclcpp::shutdown();
  return result;
}
