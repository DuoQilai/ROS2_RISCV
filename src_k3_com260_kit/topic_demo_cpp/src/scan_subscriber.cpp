#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanSubscriber : public rclcpp::Node
{
public:
  ScanSubscriber() : Node("scan_subscriber")
  {
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
        RCLCPP_INFO(get_logger(), "angle_min=%.6f angle_max=%.6f ranges=%zu frame=%s",
          message->angle_min, message->angle_max, message->ranges.size(),
          message->header.frame_id.c_str());
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanSubscriber>());
  rclcpp::shutdown();
  return 0;
}
