#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;
volatile std::sig_atomic_t stopped = 0;

class SlamMapRunner : public rclcpp::Node {
public:
  SlamMapRunner() : Node("slam_map_runner") {
    commands = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      position = msg->pose.pose.position;
      if (!start) { start = position; }
    });
    map = create_subscription<nav_msgs::msg::OccupancyGrid>("/map",
      rclcpp::QoS(1).reliable().transient_local(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (!last_stamp || *last_stamp != stamp) { ++updates; last_stamp = stamp; }
        const auto known = std::count_if(msg->data.begin(), msg->data.end(), [](int n) { return n >= 0; });
        if (!first_known) { first_known = known; }
        growth = std::max(growth, known - *first_known);
        coverage = msg->data.empty() ? 0.0 : 100.0 * known / msg->data.size();
        if (updates % 5 == 0) {
          const auto occupied = std::count(msg->data.begin(), msg->data.end(), 100);
          const auto free = std::count(msg->data.begin(), msg->data.end(), 0);
          RCLCPP_INFO(get_logger(), "[更新 #%d] 已知: %ld, 占据: %ld, 空闲: %ld, 探索率: %.1f%%",
            updates, known, occupied, free, coverage);
        }
      });
  }
  bool ready() const { return start && first_known; }
  double distance() const { return start ? std::hypot(position.x-start->x, position.y-start->y) : 0; }
  bool complete() const { return ready() && updates >= 2 && distance() > 0.2 && growth > 100; }
  void drive(double elapsed) {
    geometry_msgs::msg::Twist cmd;
    const double phase = std::fmod(elapsed, 8.0);
    if (phase < 2) { cmd.angular.z = 0.6; }
    else if (phase < 4.5) { cmd.linear.x = 0.25; }
    else if (phase < 6.5) { cmd.angular.z = -0.6; }
    else { cmd.linear.x = 0.25; }
    commands->publish(cmd);
  }
  void stop() { commands->publish(geometry_msgs::msg::Twist()); }
  void report() {
    RCLCPP_INFO(get_logger(), "map_updates=%d known_cell_growth=%ld displacement=%.3f coverage=%.1f%%",
      updates, growth, distance(), coverage);
  }
private:
  int updates = 0;
  long growth = 0;
  double coverage = 0;
  geometry_msgs::msg::Point position;
  std::optional<geometry_msgs::msg::Point> start;
  std::optional<long> first_known, last_stamp;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commands;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) { stopped = 1; });
  std::signal(SIGTERM, [](int) { stopped = 1; });
  auto node = std::make_shared<SlamMapRunner>();
  const auto deadline = std::chrono::steady_clock::now() + 180s;
  std::optional<rclcpp::Time> motion_start;
  while (!stopped && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    if (node->ready()) {
      if (!motion_start) { motion_start = node->now(); }
      const double elapsed = (node->now() - *motion_start).seconds();
      if (node->complete() || elapsed >= 25) { break; }
      node->drive(elapsed);
    }
    rclcpp::sleep_for(50ms);
  }
  for (int i = 0; i < 5; ++i) { node->stop(); rclcpp::sleep_for(100ms); }
  node->report();
  const bool ok = node->complete() && !stopped;
  if (ok) { RCLCPP_INFO(node->get_logger(), "slam-map-updated"); }
  else { RCLCPP_WARN(node->get_logger(), "建图检查未完成，已发送停车指令"); }
  rclcpp::shutdown();
  return ok ? 0 : (stopped ? 130 : 1);
}
