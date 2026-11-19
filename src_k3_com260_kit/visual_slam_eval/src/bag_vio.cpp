#include <algorithm>
#include <filesystem>
#include <iostream>
#include <vector>
#include <System.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

template<class T> T decode(const rosbag2_storage::SerializedBagMessage & record) {
  rclcpp::SerializedMessage data(*record.serialized_data);
  T message;
  rclcpp::Serialization<T>().deserialize_message(&data, &message);
  return message;
}

double stamp(const std_msgs::msg::Header & header) { return rclcpp::Time(header.stamp).seconds(); }

int main(int argc, char ** argv) {
  if (argc != 5 || std::filesystem::exists(argv[4])) {
    std::cerr << "Usage: bag_vio BAG VOCABULARY SETTINGS NEW_TRAJECTORY_FILE\n";
    return 1;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bag_vio");
  tf2_ros::TransformBroadcaster broadcaster(node);
  auto publisher = node->create_publisher<nav_msgs::msg::Path>(
    "/orb_slam3/trajectory", rclcpp::QoS(1).transient_local());
  auto camera = node->create_publisher<sensor_msgs::msg::Image>(
    "/orb_slam3/image", rclcpp::QoS(10));
  std::vector<sensor_msgs::msg::Imu> imu;
  {
    rosbag2_cpp::Reader reader;
    reader.open(argv[1]);
    while (reader.has_next()) {
      auto record = reader.read_next();
      if (record->topic_name == "/imu") imu.push_back(decode<sensor_msgs::msg::Imu>(*record));
    }
  }
  std::sort(imu.begin(), imu.end(), [](const auto & a, const auto & b) { return stamp(a.header) < stamp(b.header); });
  if (imu.empty()) {
    std::cerr << "Bag contains no /imu samples\n";
    rclcpp::shutdown();
    return 1;
  }
  ORB_SLAM3::System slam(argv[2], argv[3], ORB_SLAM3::System::IMU_MONOCULAR, false);
  rosbag2_cpp::Reader reader;
  reader.open(argv[1]);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  size_t next_imu = 0, frames = 0, tracked = 0;
  while (rclcpp::ok() && reader.has_next()) {
    auto record = reader.read_next();
    if (record->topic_name != "/camera/image_raw") continue;
    auto image = decode<sensor_msgs::msg::Image>(*record);
    camera->publish(image);
    const double time = stamp(image.header);
    std::vector<ORB_SLAM3::IMU::Point> measurements;
    while (next_imu < imu.size() && stamp(imu[next_imu].header) <= time) {
      const auto & sample = imu[next_imu++];
      const auto & a = sample.linear_acceleration;
      const auto & w = sample.angular_velocity;
      measurements.emplace_back(a.x, a.y, a.z, w.x, w.y, w.z, stamp(sample.header));
    }
    cv::Mat pixels = cv_bridge::toCvCopy(image, "mono8")->image;
    const float scale = slam.GetImageScale();
    if (scale != 1.0f) cv::resize(pixels, pixels, cv::Size(), scale, scale);
    const auto pose = slam.TrackMonocular(pixels, time, measurements).inverse();
    if (slam.GetTrackingState() == 2) {
      geometry_msgs::msg::PoseStamped point;
      point.header = image.header;
      point.header.frame_id = "map";
      const auto p = pose.translation();
      const auto q = pose.unit_quaternion();
      point.pose.position.x = p.x(); point.pose.position.y = p.y(); point.pose.position.z = p.z();
      point.pose.orientation.x = q.x(); point.pose.orientation.y = q.y();
      point.pose.orientation.z = q.z(); point.pose.orientation.w = q.w();
      geometry_msgs::msg::TransformStamped transform;
      transform.header = point.header;
      transform.child_frame_id = "camera";
      transform.transform.translation.x = p.x();
      transform.transform.translation.y = p.y();
      transform.transform.translation.z = p.z();
      transform.transform.rotation = point.pose.orientation;
      broadcaster.sendTransform(transform);
      path.header = point.header;
      path.poses.push_back(point);
      publisher->publish(path);
      ++tracked;
    }
    if (++frames % 50 == 0) std::cout << "frames=" << frames << " tracked=" << tracked << std::endl;
    rclcpp::spin_some(node);
  }
  const bool complete = rclcpp::ok();
  slam.Shutdown();
  if (tracked == 0) {
    std::cerr << "No image poses were tracked\n";
    rclcpp::shutdown();
    return 1;
  }
  slam.SaveTrajectoryEuRoC(argv[4]);
  std::cout << "frames=" << frames << " tracked=" << tracked << " imu=" << imu.size() << std::endl;
  rclcpp::shutdown();
  return complete && frames > 0 && tracked > 0 ? 0 : 1;
}
