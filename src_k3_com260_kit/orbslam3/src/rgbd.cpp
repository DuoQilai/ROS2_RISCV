#include <chrono>
#include <filesystem>
#include <functional>
#include <iostream>
#include <System.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>

using Image = sensor_msgs::msg::Image;
using Sync = message_filters::sync_policies::ApproximateTime<Image, Image>;

int main(int argc, char ** argv) {
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() != 3 || std::filesystem::exists("trajectory.tum") ||
      std::filesystem::exists("keyframes.tum") || std::filesystem::exists("rgbd-map.osa")) {
    std::cerr << "Usage in a new output directory: rgbd VOCABULARY SETTINGS\n";
    return 1;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("orbslam3_rgbd");
  tf2_ros::TransformBroadcaster broadcaster(node);
  ORB_SLAM3::System slam(args[1], args[2], ORB_SLAM3::System::RGBD, false);
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/camera_pose", 10);
  auto path_pub = node->create_publisher<nav_msgs::msg::Path>("/orb_slam3/trajectory", rclcpp::QoS(1).transient_local());
  auto map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/orb_slam3/map_points", rclcpp::QoS(1).transient_local());
  image_transport::SubscriberFilter rgb(node.get(), "/camera/color/image_raw",
    node->declare_parameter<std::string>("rgb_transport", "raw"));
  image_transport::SubscriberFilter depth(node.get(), "/camera/aligned_depth_to_color/image_raw",
    node->declare_parameter<std::string>("depth_transport", "raw"));
  message_filters::Synchronizer<Sync> sync(Sync(5), rgb, depth);
  sync.setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.02));
  nav_msgs::msg::Path path;
  size_t frames = 0, tracked = 0;
  double previous = -1;
  const auto process = [&](Image::ConstSharedPtr color, Image::ConstSharedPtr distance) {
    const double stamp = rclcpp::Time(color->header.stamp).seconds();
    if (stamp <= previous || color->width != distance->width || color->height != distance->height ||
        (distance->encoding != "32FC1" && distance->encoding != "16UC1")) {
      RCLCPP_WARN(node->get_logger(), "Rejecting nonmonotonic or incompatible RGB-D pair");
      return;
    }
    previous = stamp;
    const auto start = std::chrono::steady_clock::now();
    const auto image = cv_bridge::toCvShare(color, "rgb8");
    const auto values = cv_bridge::toCvShare(distance);
    const auto pose = slam.TrackRGBD(image->image, values->image, stamp).inverse();
    const int state = slam.GetTrackingState();
    size_t points = 0;
    if (state == 2) {
      geometry_msgs::msg::PoseStamped message;
      message.header = color->header;
      message.header.frame_id = "map";
      const auto p = pose.translation();
      const auto q = pose.unit_quaternion();
      message.pose.position.x = p.x(); message.pose.position.y = p.y(); message.pose.position.z = p.z();
      message.pose.orientation.x = q.x(); message.pose.orientation.y = q.y();
      message.pose.orientation.z = q.z(); message.pose.orientation.w = q.w();
      pose_pub->publish(message);
      geometry_msgs::msg::TransformStamped transform;
      transform.header = message.header;
      transform.child_frame_id = "slam_camera";
      transform.transform.translation.x = p.x();
      transform.transform.translation.y = p.y();
      transform.transform.translation.z = p.z();
      transform.transform.rotation = message.pose.orientation;
      broadcaster.sendTransform(transform);
      path.header = message.header;
      path.poses.push_back(message);
      path_pub->publish(path);
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header = message.header;
      const auto landmarks = slam.GetTrackedMapPoints();
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(landmarks.size());
      sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      for (const auto landmark : landmarks) {
        if (!landmark || landmark->isBad()) continue;
        const auto point = landmark->GetWorldPos();
        if (!point.allFinite()) continue;
        *x = point.x(); *y = point.y(); *z = point.z();
        ++x; ++y; ++z; ++points;
      }
      modifier.resize(points);
      map_pub->publish(cloud);
      ++tracked;
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    ++frames;
    RCLCPP_INFO(node->get_logger(), "frames=%zu tracked=%zu state=%d points=%zu stamp=%.6f processing_ms=%.3f",
      frames, tracked, state, points, stamp, ms);
  };
  sync.registerCallback(std::bind(process, std::placeholders::_1, std::placeholders::_2));
  rclcpp::spin(node);
  slam.Shutdown();
  if (tracked > 0) {
    slam.SaveTrajectoryTUM("trajectory.tum");
    slam.SaveKeyFrameTrajectoryTUM("keyframes.tum");
  }
  std::cout << "frames=" << frames << " tracked=" << tracked << std::endl;
  rclcpp::shutdown();
  return tracked > 0 ? 0 : 1;
}
