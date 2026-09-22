#include <chrono>
#include <cstdint>
#include <future>
#include <iostream>
#include <memory>
#include <string>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using namespace std::chrono_literals;

class AddTwoIntsClient : public rclcpp::Node
{
public:
  AddTwoIntsClient() : Node("add_two_ints_client")
  {
    client_ = create_client<AddTwoInts>("add_two_ints");
  }

  bool wait_for_service()
  {
    while (rclcpp::ok() && !client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "等待服务上线...");
    }
    return rclcpp::ok();
  }

  int run_sync()
  {
    auto future = client_->async_send_request(request());
    if (rclcpp::spin_until_future_complete(shared_from_this(), future, 5s) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "Result: %ld", future.get()->sum);
      return 0;
    }
    client_->remove_pending_request(future);
    RCLCPP_ERROR(get_logger(), "服务调用超时或中断");
    return 1;
  }

  void send_request_async()
  {
    client_->async_send_request(request(),
      [this](rclcpp::Client<AddTwoInts>::SharedFuture future) {
        try {
          RCLCPP_INFO(get_logger(), "Result: %ld", future.get()->sum);
          async_result_ = 0;
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "调用失败: %s", error.what());
        }
        rclcpp::shutdown();
      });
    RCLCPP_INFO(get_logger(), "异步请求已发出，响应由 spin 中的回调处理");
  }

  int run_timeout()
  {
    auto future = client_->async_send_request(request());
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      if (future.wait_for(100ms) == std::future_status::ready) {
        RCLCPP_INFO(get_logger(), "Result: %ld", future.get()->sum);
        return 0;
      }
      if (std::chrono::steady_clock::now() - start > 5s) {
        RCLCPP_ERROR(get_logger(), "服务调用超时！");
        client_->remove_pending_request(future);
        return 1;
      }
    }
    client_->remove_pending_request(future);
    return 1;
  }

  int run_retry()
  {
    for (int attempt = 0; attempt < 3; ++attempt) {
      if (client_->wait_for_service(2s) && run_sync() == 0) {
        return 0;
      }
      RCLCPP_WARN(get_logger(), "重试 %d/%d...", attempt + 1, 3);
    }
    RCLCPP_ERROR(get_logger(), "所有重试均失败！");
    return 1;
  }

  int async_result() const {return async_result_;}

private:
  static AddTwoInts::Request::SharedPtr request()
  {
    auto message = std::make_shared<AddTwoInts::Request>();
    message->a = 10;
    message->b = 20;
    return message;
  }

  rclcpp::Client<AddTwoInts>::SharedPtr client_;
  int async_result_{1};
};

int main(int argc, char ** argv)
{
  const std::string mode = argc == 1 ? "sync" : argv[1];
  if (argc > 2 || (mode != "sync" && mode != "async" && mode != "timeout" && mode != "retry")) {
    std::cerr << "Usage: teacher_client [sync|async|timeout|retry]\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsClient>();
  int result = 1;
  if (mode == "retry") {
    result = node->run_retry();
  } else if (node->wait_for_service()) {
    if (mode == "sync") {
      result = node->run_sync();
    } else if (mode == "timeout") {
      result = node->run_timeout();
    } else {
      node->send_request_async();
      rclcpp::spin(node);
      result = node->async_result();
    }
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}
