#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

class SensorMonitor : public rclcpp::Node {
public:
  SensorMonitor() : Node("sensor_monitor") {
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        ++scan_count_;
        const auto valid = std::count_if(msg->ranges.begin(), msg->ranges.end(),
          [msg](float r) { return r > 0 && r < msg->range_max; });
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "激光: %zu条射线, %td个有效点, 范围[%.2f, %.2f]",
          msg->ranges.size(), valid, msg->range_min, msg->range_max);
      });
    imu_ = create_subscription<sensor_msgs::msg::Imu>("/imu", 10,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        ++imu_count_;
        const auto & a = msg->linear_acceleration; const auto & g = msg->angular_velocity;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "IMU: acc=(%.2f,%.2f,%.2f) gyro=(%.2f,%.2f,%.2f)", a.x, a.y, a.z, g.x, g.y, g.z);
      });
    odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        ++odom_count_;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "里程计: pos=(%.2f,%.2f) vel=(%.2fm/s)",
          msg->pose.pose.position.x, msg->pose.pose.position.y, msg->twist.twist.linear.x);
      });
  }
private:
  std::size_t scan_count_ = 0, imu_count_ = 0, odom_count_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<SensorMonitor>()); rclcpp::shutdown();
}
