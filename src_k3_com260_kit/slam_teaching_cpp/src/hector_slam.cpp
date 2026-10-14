#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "slam_teaching_cpp/hector.hpp"

class HectorSLAMNode : public rclcpp::Node {
public:
  HectorSLAMNode() : Node("hector_slam"), buffer_(get_clock()), listener_(buffer_), broadcaster_(*this) {
    const auto resolution = declare_parameter("map_resolution", 0.05);
    const auto size = declare_parameter("map_size", 20.0);
    matcher_.max_iterations = declare_parameter("max_iterations", 15);
    // 沿用原示例参数声明；本教学地图更新固定使用 log-odds -0.3/+1.0。
    declare_parameter("update_factor_free", 0.4); declare_parameter("update_factor_occupied", 0.9);
    if (resolution <= 0 || size <= 2*resolution || matcher_.max_iterations <= 0) {
      throw std::invalid_argument("map_resolution/map_size/max_iterations 无效");
    }
    map_ = std::make_unique<slam_teaching::OccupancyGridMap>(size, size, resolution);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/slam_out_pose", 10);
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(*msg); });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish_map(); });
    RCLCPP_INFO(get_logger(), "Hector-SLAM教学节点已启动");
  }
private:
  void on_scan(const sensor_msgs::msg::LaserScan & msg) {
    try {
      const auto laser = buffer_.lookupTransform("base_link", msg.header.frame_id, tf2::TimePointZero);
      tf2::Transform base_from_laser; tf2::fromMsg(laser.transform, base_from_laser);
      slam_teaching::Points points;
      for (std::size_t i = 0; i < msg.ranges.size(); ++i) {
        const double r = msg.ranges[i], angle = msg.angle_min + i * msg.angle_increment;
        if (std::isfinite(r) && r > msg.range_min && r < msg.range_max) {
          auto p = base_from_laser * tf2::Vector3(r*std::cos(angle), r*std::sin(angle), 0);
          points.emplace_back(p.x(), p.y());
        }
      }
      if (points.size() < 10) { return; }
      if (first_scan_) {
        map_->update_with_scan(points, pose_); first_scan_ = false;
        RCLCPP_INFO(get_logger(), "首帧扫描已处理"); return;
      }
      pose_ = matcher_.match(points, map_->get_probability_map(), map_->resolution, pose_, map_->origin);
      map_->update_with_scan(points, pose_);
      geometry_msgs::msg::PoseStamped output;
      output.header.frame_id = "map"; output.header.stamp = msg.header.stamp;
      output.pose.position.x = pose_.x(); output.pose.position.y = pose_.y();
      output.pose.orientation.z = std::sin(pose_.z()/2); output.pose.orientation.w = std::cos(pose_.z()/2);
      pose_pub_->publish(output);
      // map→odom = map→base_link × inverse(odom→base_link)。
      const auto odom = buffer_.lookupTransform("odom", "base_link", rclcpp::Time(msg.header.stamp));
      tf2::Transform odom_from_base; tf2::fromMsg(odom.transform, odom_from_base);
      tf2::Transform map_from_base; tf2::fromMsg(output.pose, map_from_base);
      geometry_msgs::msg::TransformStamped tf;
      tf.header = output.header; tf.child_frame_id = "odom";
      tf.transform = tf2::toMsg(map_from_base * odom_from_base.inverse()); broadcaster_.sendTransform(tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF等待: %s", e.what());
    }
  }
  void publish_map() {
    nav_msgs::msg::OccupancyGrid msg; msg.header.frame_id = "map"; msg.header.stamp = now();
    msg.info.resolution = map_->resolution; msg.info.width = map_->width_pixels; msg.info.height = map_->height_pixels;
    msg.info.origin.position.x = map_->origin.x(); msg.info.origin.position.y = map_->origin.y();
    msg.info.origin.orientation.w = 1.0; msg.data = map_->get_occupancy_grid(); map_pub_->publish(msg);
  }
  slam_teaching::Vector3d pose_ = slam_teaching::Vector3d::Zero(); bool first_scan_ = true;
  std::unique_ptr<slam_teaching::OccupancyGridMap> map_; slam_teaching::HectorScanMatcher matcher_;
  tf2_ros::Buffer buffer_; tf2_ros::TransformListener listener_; tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<HectorSLAMNode>()); rclcpp::shutdown();
}
