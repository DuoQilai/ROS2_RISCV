#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
class StaticTFPublisher : public rclcpp::Node
{
public:
  StaticTFPublisher() : Node("static_tf_pub")
  {
    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now(); t.header.frame_id = "base_link"; t.child_frame_id = "laser_frame";
    t.transform.translation.x = declare_parameter("x", 0.2);
    t.transform.translation.z = declare_parameter("z", 0.1);
    t.transform.rotation.w = 1.0;
    broadcaster_->sendTransform(t);
  }
private:
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<StaticTFPublisher>());
  rclcpp::shutdown(); return 0;
}
