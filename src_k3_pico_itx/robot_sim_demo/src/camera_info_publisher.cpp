#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class CameraInfoPublisher : public rclcpp::Node
{
public:
  CameraInfoPublisher() : Node("camera_info_publisher")
  {
    const auto topic = declare_parameter<std::string>("topic", "/camera/camera_info");
    info_.header.frame_id = declare_parameter<std::string>("frame_id", "camera_link");
    info_.width = declare_parameter<int>("width", 320);
    info_.height = declare_parameter<int>("height", 180);
    const auto fov = declare_parameter<double>("horizontal_fov", 1.0472);
    const auto rate = std::max(declare_parameter<double>("publish_rate", 3.0), 0.1);
    const double focal = (info_.width * 0.5) / std::tan(fov * 0.5);
    const double cx = (info_.width - 1.0) * 0.5;
    const double cy = (info_.height - 1.0) * 0.5;
    info_.distortion_model = "plumb_bob";
    info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    info_.k = {focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0};
    info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info_.p = {focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(topic, 10);
    clock_subscription_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", 10, [this](rosgraph_msgs::msg::Clock::ConstSharedPtr message) {
        latest_sim_time_ = message->clock;
      });
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
      [this]() {
        info_.header.stamp = latest_sim_time_.value_or(now());
        publisher_->publish(info_);
      });
  }

private:
  sensor_msgs::msg::CameraInfo info_;
  std::optional<builtin_interfaces::msg::Time> latest_sim_time_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}
