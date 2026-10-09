#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
using namespace std::chrono_literals;
class TFListener : public rclcpp::Node
{
public:
  TFListener() : Node("tf_listener"), buffer_(get_clock()), listener_(buffer_)
  {
    timer_ = create_wall_timer(1s, [this]() {
      if (!buffer_.canTransform("base_link", "laser_frame", tf2::TimePointZero, tf2::durationFromSec(2.0))) {
        RCLCPP_WARN(get_logger(), "等待 laser_frame 变换超时"); return;
      }
      try {
        auto t = buffer_.lookupTransform("base_link", "laser_frame", tf2::TimePointZero);
        const auto & p = t.transform.translation;
        const auto & q = t.transform.rotation;
        RCLCPP_INFO(get_logger(), "Laser→Base: pos=(%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f)",
          p.x, p.y, p.z, q.x, q.y, q.z, q.w);
        geometry_msgs::msg::PointStamped point, camera;
        point.header.frame_id = "laser_frame"; point.point.x = 1.0; point.point.y = 0.5;
        const auto transform = buffer_.lookupTransform("camera_frame", "laser_frame", tf2::TimePointZero);
        tf2::doTransform(point, camera, transform);
        RCLCPP_INFO(get_logger(), "激光点 (1.0,0.5,0.0) → 相机系: (%.3f, %.3f, %.3f)",
          camera.point.x, camera.point.y, camera.point.z);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN(get_logger(), "坐标变换失败: %s", e.what());
      }
    });
  }
private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<TFListener>());
  rclcpp::shutdown(); return 0;
}
