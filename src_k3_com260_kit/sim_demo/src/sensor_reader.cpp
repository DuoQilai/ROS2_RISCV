#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SensorReader : public rclcpp::Node
{
public:
  SensorReader() : Node("sensor_reader")
  {
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        float nearest = std::numeric_limits<float>::infinity(), farthest = 0.0F;
        for (const float range : msg->ranges) {
          if (msg->range_min < range && range < msg->range_max) {
            nearest = std::min(nearest, range);
            farthest = std::max(farthest, range);
          }
        }
        if (std::isfinite(nearest)) {
          RCLCPP_INFO(get_logger(), "LiDAR: %.2frad, %zu 点, 最近: %.2fm, 最远: %.2fm",
            msg->angle_max - msg->angle_min, msg->ranges.size(), nearest, farthest);
        }
      });
    image_ = create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Camera: %ux%u, 编码: %s",
          msg->width, msg->height, msg->encoding.c_str());
      });
  }
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorReader>());
  rclcpp::shutdown();
  return 0;
}
