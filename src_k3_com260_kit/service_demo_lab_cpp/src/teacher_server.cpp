#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsServer : public rclcpp::Node
{
public:
  AddTwoIntsServer() : Node("add_two_ints_server")
  {
    service_ = create_service<AddTwoInts>("add_two_ints",
      [this](const AddTwoInts::Request::SharedPtr request,
        AddTwoInts::Response::SharedPtr response) {
        response->sum = request->a + request->b;
        RCLCPP_INFO(get_logger(), "收到请求: %ld + %ld = %ld",
          request->a, request->b, response->sum);
      });
  }
private:
  rclcpp::Service<AddTwoInts>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddTwoIntsServer>());
  rclcpp::shutdown();
  return 0;
}
