#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class AmclEvaluator : public rclcpp::Node
{
public:
  AmclEvaluator() : Node("amcl_evaluator"), buffer_(get_clock()), listener_(buffer_)
  {
    const auto topic = declare_parameter<std::string>("ground_truth_topic", "/ground_truth/odom");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {amcl_ = msg;});
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(topic, 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {truth_ = msg;});
    timer_ = create_wall_timer(1s, [this]() {evaluate();});
    RCLCPP_INFO(get_logger(), "Comparing /amcl_pose with %s in %s", topic.c_str(), target_frame_.c_str());
  }
private:
  static double yaw(const geometry_msgs::msg::Pose & pose)
  {
    const auto & q = pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
  geometry_msgs::msg::PoseStamped transform(
    const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose)
  {
    geometry_msgs::msg::PoseStamped input;
    input.header = header;
    input.pose = pose;
    return buffer_.transform(input, target_frame_, tf2::durationFromSec(0.2));
  }
  void evaluate()
  {
    if (!amcl_ || !truth_) {return;}
    try {
      const auto estimated = transform(amcl_->header, amcl_->pose.pose);
      const auto actual = transform(truth_->header, truth_->pose.pose);
      const double position_error = std::hypot(estimated.pose.position.x - actual.pose.position.x,
        estimated.pose.position.y - actual.pose.position.y);
      const double delta = yaw(estimated.pose) - yaw(actual.pose);
      const double yaw_error = std::abs(std::atan2(std::sin(delta), std::cos(delta))) * 180.0 / std::acos(-1.0);
      RCLCPP_INFO(get_logger(), "position_error=%.3f m, yaw_error=%.2f deg", position_error, yaw_error);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(get_logger(), "Cannot transform poses: %s", error.what());
    }
  }
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr amcl_;
  nav_msgs::msg::Odometry::ConstSharedPtr truth_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclEvaluator>());
  rclcpp::shutdown();
  return 0;
}
