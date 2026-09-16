#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class AddServer : public rclcpp::Node
{
public:
  AddServer()
  : Node("add_two_ints_server")
  {
    declare_parameter<double>("delay_sec", 3.0);
    service_ = create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&AddServer::handle, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "ADD_SERVER_READY");
  }

private:
  void handle(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    const auto delay = get_parameter("delay_sec").as_double();
    RCLCPP_INFO(get_logger(), "ADD_REQUEST a=%ld b=%ld delay=%.1f", request->a, request->b, delay);
    std::this_thread::sleep_for(std::chrono::duration<double>(delay));
    response->sum = request->a + request->b;
    RCLCPP_INFO(get_logger(), "ADD_RESULT sum=%ld", response->sum);
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddServer>());
  rclcpp::shutdown();
  return 0;
}
