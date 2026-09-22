#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/msg/person.hpp"

class PersonSubscriber : public rclcpp::Node
{
public:
  PersonSubscriber() : Node("person_subscriber")
  {
    subscription_ = create_subscription<sensor_interfaces::msg::Person>(
      "/person_info", 10, [this](sensor_interfaces::msg::Person::ConstSharedPtr message) {
        RCLCPP_INFO(get_logger(), "姓名=%s, 年龄=%d, 身高=%.2fm",
          message->name.c_str(), message->age, message->height);
      });
  }

private:
  rclcpp::Subscription<sensor_interfaces::msg::Person>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PersonSubscriber>());
  rclcpp::shutdown();
  return 0;
}
