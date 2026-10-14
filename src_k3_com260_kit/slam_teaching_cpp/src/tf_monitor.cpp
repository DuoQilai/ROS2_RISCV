#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TFMonitor : public rclcpp::Node {
public:
  TFMonitor() : Node("tf_monitor"), buffer_(get_clock()), listener_(buffer_) {
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { query_tf(); });
  }
  bool can_transform_check(const std::string & source, const std::string & target) {
    return buffer_.canTransform(target, source, tf2::TimePointZero, tf2::durationFromSec(0.1));
  }
private:
  void query_tf() {
    try {
      const auto trans = buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      RCLCPP_INFO(get_logger(), "map→base_link: (%.3f, %.3f)",
        trans.transform.translation.x, trans.transform.translation.y);
      geometry_msgs::msg::PointStamped point;
      point.header.frame_id = "laser_link"; point.point.x = 1.0;
      const auto mapped = buffer_.transform(point, "map");
      RCLCPP_INFO(get_logger(), "激光点(1,0,0)在map下: (%.3f, %.3f)", mapped.point.x, mapped.point.y);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "TF查询失败: %s", e.what());
    }
  }
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<TFMonitor>()); rclcpp::shutdown();
}
