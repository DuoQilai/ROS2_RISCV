#include <chrono>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
using namespace std::chrono_literals;
class DynamicTFPublisher : public rclcpp::Node
{
public:
  DynamicTFPublisher() : Node("dynamic_tf_pub")
  {
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(100ms, [this]() {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now(); t.header.frame_id = "odom"; t.child_frame_id = "base_link";
      t.transform.translation.x = std::sin(count_ * 0.1);
      t.transform.translation.y = std::cos(count_ * 0.1);
      t.transform.rotation.w = 1.0;
      broadcaster_->sendTransform(t); ++count_;
    });
  }
private:
  int count_{0};
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<DynamicTFPublisher>());
  rclcpp::shutdown(); return 0;
}
