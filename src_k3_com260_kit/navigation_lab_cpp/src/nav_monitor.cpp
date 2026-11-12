#include <array>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class NavMonitor : public rclcpp::Node {
public:
  NavMonitor() : Node("nav_monitor") {
    plan_ = create_subscription<nav_msgs::msg::Path>("/plan", 10, [this](const nav_msgs::msg::Path & msg) {
      if (msg.poses.empty()) { return; }
      const auto & first = msg.poses.front().pose.position;
      const auto & last = msg.poses.back().pose.position;
      const std::array<double, 4> key{first.x, first.y, last.x, last.y};
      if (count_ == 0 || key != last_key_) {
        RCLCPP_INFO(get_logger(), "New global plan #%u", ++count_);
        last_key_ = key;
      }
    });
    local_ = create_subscription<nav_msgs::msg::Path>("/local_plan", 10, [this](const nav_msgs::msg::Path & msg) {
      RCLCPP_INFO(get_logger(), "Local plan: %zu poses", msg.poses.size());
    });
    cmd_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, [this](const geometry_msgs::msg::Twist & msg) {
      RCLCPP_INFO(get_logger(), "Cmd vel: linear=%.2f angular=%.2f", msg.linear.x, msg.angular.z);
    });
    odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS(),
      [](const nav_msgs::msg::Odometry &) {});
  }
private:
  unsigned count_ = 0;
  std::array<double, 4> last_key_{};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_, local_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavMonitor>());
  rclcpp::shutdown();
}
