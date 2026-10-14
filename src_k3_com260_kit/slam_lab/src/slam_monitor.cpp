#include <algorithm>
#include <cstdint>
#include <memory>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class SlamMonitor : public rclcpp::Node
{
public:
  SlamMonitor() : Node("slam_monitor")
  {
    subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        const auto occupied = std::count_if(msg->data.begin(), msg->data.end(),
          [](int8_t value) {return value >= 65;});
        const auto unknown = std::count_if(msg->data.begin(), msg->data.end(),
          [](int8_t value) {return value < 0;});
        const auto known = msg->data.size() - unknown;
        const double ratio = msg->data.empty() ? 0.0 : 100.0 * known / msg->data.size();
        RCLCPP_INFO(get_logger(),
          "#%zu: %ux%u, %.3f m/cell, occupied=%zu, free=%zu, unknown=%zu, known=%.1f%%",
          ++count_, msg->info.width, msg->info.height, msg->info.resolution,
          static_cast<size_t>(occupied), known - occupied, static_cast<size_t>(unknown), ratio);
      });
    RCLCPP_INFO(get_logger(), "SLAM map monitor started");
  }
private:
  size_t count_{0};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamMonitor>());
  rclcpp::shutdown();
  return 0;
}
