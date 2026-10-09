#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("time_query");
  const auto target = node->declare_parameter("target", "base_link");
  const auto source = node->declare_parameter("source", "laser_frame");
  const double history = node->declare_parameter("history_sec", 0.0);
  tf2_ros::Buffer buffer(node->get_clock());
  tf2_ros::TransformListener listener(buffer);
  // The listener owns its subscription thread while the main thread waits.
  if (history > 0.0) {rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(history + 1.0)));}
  int result = 1;
  for (int attempt = 1; attempt <= 3 && rclcpp::ok(); ++attempt) {
    const auto stamp = history > 0.0 ? node->now() - rclcpp::Duration::from_seconds(history) : rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
    if (!buffer.canTransform(target, source, stamp, rclcpp::Duration::from_seconds(2.0))) {
      RCLCPP_WARN(node->get_logger(), "等待变换超时，尝试 %d/3", attempt); continue;
    }
    try {
      const auto t = buffer.lookupTransform(target, source, stamp);
      geometry_msgs::msg::PointStamped point, converted;
      point.header.frame_id = source; point.point.x = 1.0;
      tf2::doTransform(point, converted, t);
      RCLCPP_INFO(node->get_logger(), "history=%.1f translation=(%.6f,%.6f,%.6f) point=(%.6f,%.6f,%.6f)",
        history, t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
        converted.point.x, converted.point.y, converted.point.z);
      result = 0; break;
    } catch (const tf2::TransformException & e) {RCLCPP_WARN(node->get_logger(), "%s", e.what());}
  }
  rclcpp::shutdown(); return result;
}
