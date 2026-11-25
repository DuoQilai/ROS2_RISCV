#include <algorithm>
#include <message_filters/subscriber.h>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class SensorSync : public rclcpp::Node {
  using Imu = sensor_msgs::msg::Imu;
  using Scan = sensor_msgs::msg::LaserScan;
  using Image = sensor_msgs::msg::Image;
  using Policy = message_filters::sync_policies::ApproximateTime<Imu, Scan, Image>;
public:
  SensorSync() : Node("sensor_sync"),
      imu(this, "/imu", rmw_qos_profile_sensor_data),
      scan(this, "/scan", rmw_qos_profile_sensor_data),
      sync(Policy(declare_parameter("sync_queue_size", 10)), imu, scan, image) {
    image.subscribe(this, "/camera/image_raw", declare_parameter<std::string>("image_transport", "compressed"), rmw_qos_profile_sensor_data);
    sync.setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
    sync.registerCallback(&SensorSync::synchronized, this);
  }
private:
  void synchronized(const Imu::ConstSharedPtr & a, const Scan::ConstSharedPtr & b,
                    const Image::ConstSharedPtr & c) {
    const double ti = rclcpp::Time(a->header.stamp).seconds();
    const double tl = rclcpp::Time(b->header.stamp).seconds();
    const double tc = rclcpp::Time(c->header.stamp).seconds();
    const double spread = std::max({ti, tl, tc}) - std::min({ti, tl, tc});
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "传感器同步: IMU=%.3f LiDAR=%.3f Camera=%.3f spread=%.3fms", ti, tl, tc, spread*1000);
  }
  message_filters::Subscriber<Imu> imu;
  message_filters::Subscriber<Scan> scan;
  image_transport::SubscriberFilter image;
  message_filters::Synchronizer<Policy> sync;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSync>());
  rclcpp::shutdown();
}
