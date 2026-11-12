#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include "grid_path_planner/search.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("global_planner_demo");
  const Point start{node->declare_parameter("start_x", 0.0), node->declare_parameter("start_y", 0.0)};
  const Point goal{node->declare_parameter("goal_x", 3.0), node->declare_parameter("goal_y", 2.0)};
  auto publisher = node->create_publisher<nav_msgs::msg::Path>("planned_path", rclcpp::QoS(1).transient_local());
  bool finished = false, success = false;
  auto subscription = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/global_costmap/costmap", rclcpp::QoS(1).transient_local(),
    [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
      if (finished) { return; }
      const auto & info = message->info;
      if (!info.width || !info.height || info.resolution <= 0 ||
          message->data.size() != info.width * info.height ||
          std::abs(info.origin.orientation.w - 1) > 1e-6) { return; }
      Costmap2D map(info.width * double(info.resolution), info.height * double(info.resolution), info.resolution);
      map.width = info.width; map.height = info.height;
      map.data.resize(message->data.size());
      map.origin_x = info.origin.position.x;
      map.origin_y = info.origin.position.y;
      for (std::size_t i = 0; i < message->data.size(); ++i) {
        const int value = message->data[i];
        map.data[i] = value < 0 ? 255 : value == 100 ? 254 : value == 99 ? 253 : value * 252 / 98;
      }
      const auto result = search_grid(map, start, goal, true);
      nav_msgs::msg::Path path;
      path.header = message->header;
      for (auto [x, y] : result.path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = x; pose.pose.position.y = y;
        pose.pose.orientation.w = 1;
        path.poses.push_back(pose);
      }
      publisher->publish(path);
      success = !path.poses.empty();
      finished = true;
      RCLCPP_INFO(node->get_logger(), "A* 路径: %zu 点，代价=%.3f，扩展=%d",
        path.poses.size(), result.cost, result.expanded);
    });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (rclcpp::ok() && !finished && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
  rclcpp::sleep_for(std::chrono::seconds(1));
  if (!success) { RCLCPP_ERROR(node->get_logger(), "没有获得有效地图或可行路径"); }
  rclcpp::shutdown();
  return success ? 0 : 1;
}
