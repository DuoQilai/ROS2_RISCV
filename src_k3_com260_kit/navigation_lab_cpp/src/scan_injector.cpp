#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanInjector : public rclcpp::Node {
public:
  ScanInjector() : Node("scan_injector") {
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan_injected", 10);
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.1),
      [this] { publish_scan(); });
  }
private:
  void publish_scan() {
    angle_ += 0.02;
    const double pi = std::acos(-1.0);
    const double x = 2.0 + 0.8 * std::sin(angle_ * 0.5), y = 0.8 * std::cos(angle_ * 0.5);
    const double distance = std::hypot(x, y), bearing = std::atan2(y, x);
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = "laser_link";
    scan.header.stamp = now();
    scan.angle_min = -pi;
    scan.angle_max = pi;
    scan.angle_increment = pi / 180;
    scan.time_increment = 1.0 / 360 / 180;
    scan.scan_time = 0.1;
    scan.range_min = 0.1;
    scan.range_max = 3.5;
    scan.ranges.assign(360, 3.5);
    scan.intensities.assign(360, 0.0);
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double theta = -pi + i * pi / 180;
      if (std::abs(theta - bearing) < 0.05 && distance > 0.1 && distance < 3.0) {
        scan.ranges[i] = distance;
      }
    }
    publisher_->publish(scan);
  }
  double angle_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanInjector>());
  rclcpp::shutdown();
}
