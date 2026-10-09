#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
using namespace std::chrono_literals;
class TfLookup : public rclcpp::Node
{
public:
  TfLookup() : Node("tf_lookup"), buffer_(get_clock()), listener_(buffer_)
  {
    timer_ = create_wall_timer(2s, [this]() {
      for (const char * frame : {"laser_link", "camera_link"}) {
        try {
          const auto t = buffer_.lookupTransform("base_link", frame, tf2::TimePointZero);
          const auto & p = t.transform.translation;
          RCLCPP_INFO(get_logger(), "%s 位置: x=%.3f, y=%.3f, z=%.3f", frame, p.x, p.y, p.z);
        } catch (const tf2::TransformException & e) {
          RCLCPP_WARN(get_logger(), "%s TF查询失败: %s", frame, e.what());
        }
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
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<TfLookup>());
  rclcpp::shutdown(); return 0;
}
