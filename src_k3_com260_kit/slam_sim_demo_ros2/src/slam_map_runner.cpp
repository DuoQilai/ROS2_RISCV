#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}

class SlamMapRunner : public rclcpp::Node
{
public:
  SlamMapRunner() : Node("slam_map_runner")
  {
    timeout_ = declare_parameter("timeout_sec", 60.0);
    if (!std::isfinite(timeout_) || timeout_ <= 0.0) {throw std::invalid_argument("timeout_sec must be finite and positive");}
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        latest_ = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
        if (!start_) {start_ = latest_;}
      });
    map_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (!last_stamp_ || stamp != *last_stamp_) {++map_updates_; last_stamp_ = stamp;}
        const size_t known = std::count_if(msg->data.begin(), msg->data.end(), [](int8_t value) {return value >= 0;});
        if (!first_known_) {first_known_ = known;}
        max_known_ = std::max(max_known_, known);
      });
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        ++scan_updates_;
        const size_t finite = std::count_if(msg->ranges.begin(), msg->ranges.end(), [](float value) {return std::isfinite(value);});
        max_finite_ = std::max(max_finite_, finite);
      });
  }
  bool ready() const {return latest_.has_value() && first_known_.has_value();}
  double timeout() const {return timeout_;}
  double distance() const {return start_ && latest_ ? std::hypot(latest_->first - start_->first, latest_->second - start_->second) : 0.0;}
  bool succeeded() const {return ready() && map_updates_ >= 2 && scan_updates_ >= 2 && distance() > 0.15 && max_known_ - *first_known_ > 20;}
  void command(double elapsed)
  {
    geometry_msgs::msg::Twist msg;
    const double phase = std::fmod(elapsed, 20.0);
    if (phase < 8.0 || phase >= 12.0) {msg.linear.x = 0.18;}
    else {msg.angular.z = 0.5;}
    publisher_->publish(msg);
  }
  void stop() {publisher_->publish(geometry_msgs::msg::Twist());}
  void metrics() const
  {
    RCLCPP_INFO(get_logger(), "map_updates=%zu, known_cell_growth=%zu, odom_distance=%.3f, scan_updates=%zu, max_finite_ranges=%zu",
      map_updates_, max_known_ - first_known_.value_or(0), distance(), scan_updates_, max_finite_);
  }
private:
  double timeout_;
  std::optional<std::pair<double, double>> start_, latest_;
  std::optional<size_t> first_known_;
  std::optional<int64_t> last_stamp_;
  size_t max_known_{0}, map_updates_{0}, scan_updates_{0}, max_finite_{0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  int status = 1;
  try {
    auto node = std::make_shared<SlamMapRunner>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto started = std::chrono::steady_clock::now();
    std::optional<std::chrono::steady_clock::time_point> motion_start;
    while (rclcpp::ok() && !stop_requested && std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() < node->timeout()) {
      executor.spin_once(100ms);
      if (!node->ready()) {continue;}
      if (!motion_start) {motion_start = std::chrono::steady_clock::now();}
      node->command(std::chrono::duration<double>(std::chrono::steady_clock::now() - *motion_start).count());
      if (node->succeeded()) {status = 0; break;}
    }
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {node->stop(); std::this_thread::sleep_for(100ms);}
    node->metrics();
    if (status == 0) {RCLCPP_INFO(node->get_logger(), "slam-map-updated");}
    else if (stop_requested) {RCLCPP_WARN(node->get_logger(), "Map check interrupted; stop command sent"); status = 130;}
    else {RCLCPP_ERROR(node->get_logger(), "slam_toolbox did not publish a growing map after driving the robot");}
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("slam_map_runner"), "%s", error.what());
  }
  rclcpp::shutdown();
  return status;
}
