#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/msg/sensor_data.hpp"

using namespace std::chrono_literals;

class SensorPublisher : public rclcpp::Node
{
public:
  SensorPublisher()
  : Node("sensor_publisher")
  {
    publisher_ = create_publisher<sensor_interfaces::msg::SensorData>("/sensor_data", 10);
    timer_ = create_wall_timer(1s, std::bind(&SensorPublisher::publish, this));
  }

private:
  void publish()
  {
    sensor_interfaces::msg::SensorData message;
    message.temperature = 25.5F;
    message.humidity = 60.0F;
    message.pressure = 1013.25F;
    message.device_id = "sensor_01";
    publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "SENSOR_PUBLISHED device_id=%s", message.device_id.c_str());
  }

  rclcpp::Publisher<sensor_interfaces::msg::SensorData>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorPublisher>());
  rclcpp::shutdown();
  return 0;
}
