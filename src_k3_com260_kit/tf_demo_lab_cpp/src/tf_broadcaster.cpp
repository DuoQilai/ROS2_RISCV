#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

class TFBroadcaster : public rclcpp::Node
{
public:
  TFBroadcaster() : Node("tf_broadcaster")
  {
    const double rate = declare_parameter("rate_hz", 20.0);
    step_ = declare_parameter("angle_step", 0.05);
    if (!std::isfinite(rate) || rate <= 0.0 || !std::isfinite(step_)) {
      throw std::invalid_argument("rate_hz must be positive and angle_step finite");
    }
    dynamic_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    auto add = [&](const char * child, double x, double y, double z) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now(); t.header.frame_id = "base_link"; t.child_frame_id = child;
      t.transform.translation.x = x; t.transform.translation.y = y; t.transform.translation.z = z;
      t.transform.rotation.w = 1.0; transforms.push_back(t);
    };
    add("laser_frame", 0.2, 0.0, 0.1);
    add("camera_frame", 0.15, 0.0, 0.25);
    add("imu_link", 0.0, 0.0, 0.05);
    add("left_wheel", 0.0, 0.15, -0.05);
    add("right_wheel", 0.0, -0.15, -0.05);
    static_->sendTransform(transforms);
    RCLCPP_INFO(get_logger(), "已发送 5 个静态 TF 变换；动态频率 %.1f Hz，角步长 %.3f rad", rate, step_);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now(); t.header.frame_id = "odom"; t.child_frame_id = "base_link";
      t.transform.translation.x = std::cos(angle_); t.transform.translation.y = std::sin(angle_);
      t.transform.rotation.z = std::sin(angle_ / 2.0); t.transform.rotation.w = std::cos(angle_ / 2.0);
      dynamic_->sendTransform(t); angle_ += step_;
    });
  }
private:
  double angle_{0.0}, step_{0.05};
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
