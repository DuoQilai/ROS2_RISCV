#include <array>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class CostmapDebugger : public rclcpp::Node {
public:
  CostmapDebugger() : Node("costmap_debugger") {
    const std::array<std::string, 2> topics{"/global_costmap/costmap", "/local_costmap/costmap"};
    for (std::size_t i = 0; i < topics.size(); ++i) {
      subscriptions_[i] = create_subscription<nav_msgs::msg::OccupancyGrid>(
        topics[i], rclcpp::QoS(1).transient_local().reliable(),
        [this, topic = topics[i]](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
          if (map->data.empty()) { return; }
          std::array<std::size_t, 5> bins{};
          for (int value : map->data) {
            if (value == -1) { ++bins[0]; }
            else if (value == 0) { ++bins[1]; }
            else if (value == 99) { ++bins[3]; }
            else if (value == 100) { ++bins[4]; }
            else { ++bins[2]; }
          }
          const double percent = 100.0 / map->data.size();
          RCLCPP_INFO(get_logger(), "%s %ux%u resolution=%.3f 未知=%.1f%% 空闲=%.1f%% 膨胀=%.1f%% 内切=%.1f%% 致命=%.1f%%",
            topic.c_str(), map->info.width, map->info.height, map->info.resolution,
            bins[0] * percent, bins[1] * percent, bins[2] * percent, bins[3] * percent, bins[4] * percent);
        });
    }
  }
private:
  std::array<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr, 2> subscriptions_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapDebugger>());
  rclcpp::shutdown();
}
