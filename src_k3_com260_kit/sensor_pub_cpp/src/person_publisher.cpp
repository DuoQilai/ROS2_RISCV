#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/msg/person.hpp"

class PersonPublisher : public rclcpp::Node
{
public:
  PersonPublisher() : Node("person_publisher")
  {
    publisher_ = create_publisher<sensor_interfaces::msg::Person>("/person_info", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      sensor_interfaces::msg::Person message;
      message.name = "Li Ming";
      message.age = 20;
      message.height = 1.75;
      publisher_->publish(message);
      RCLCPP_INFO(get_logger(), "发布: %s", message.name.c_str());
    });
  }

private:
  rclcpp::Publisher<sensor_interfaces::msg::Person>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PersonPublisher>());
  rclcpp::shutdown();
  return 0;
}
