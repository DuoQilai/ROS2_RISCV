#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "costmap_demo/costmap.hpp"

class ObstacleLayerProcessor : public rclcpp::Node {
public:
  ObstacleLayerProcessor() : Node("obstacle_layer_processor"), buffer_(get_clock()), listener_(buffer_) {
    std::fill(map_.data.begin(), map_.data.end(), 0);
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/teaching_obstacles", rclcpp::QoS(1).transient_local());
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
        try {
          const auto transform = buffer_.lookupTransform("map", scan->header.frame_id, scan->header.stamp);
          auto origin = point(0, 0, 0, transform);
          std::vector<geometry_msgs::msg::PointStamped> hits;
          for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
            const double range = scan->ranges[i];
            if (!std::isfinite(range) || range <= scan->range_min || range >= scan->range_max) { continue; }
            const double angle = scan->angle_min + i * scan->angle_increment;
            auto hit = point(range * std::cos(angle), range * std::sin(angle), 0, transform);
            for (double distance = 0; distance < range - map_.resolution; distance += map_.resolution / 2) {
              const double fraction = distance / range;
              map_.set_cost(origin.point.x + fraction * (hit.point.x - origin.point.x),
                origin.point.y + fraction * (hit.point.y - origin.point.y), 0);
            }
            hits.push_back(hit);
          }
          // Clear all rays before marking endpoints so crossing rays cannot erase new hits.
          for (const auto & hit : hits) { map_.set_cost(hit.point.x, hit.point.y, 254); }
          publish();
        } catch (const tf2::TransformException & error) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
        }
      });
    cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>("/point_cloud", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        try {
          const auto transform = buffer_.lookupTransform("map", cloud->header.frame_id, cloud->header.stamp);
          sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y"), z(*cloud, "z");
          for (; x != x.end(); ++x, ++y, ++z) {
            if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) { continue; }
            auto hit = point(*x, *y, *z, transform);
            if (hit.point.z > -0.1 && hit.point.z < 2.0) { map_.set_cost(hit.point.x, hit.point.y, 254); }
          }
          publish();
        } catch (const std::runtime_error & error) {
          RCLCPP_WARN(get_logger(), "点云处理失败: %s", error.what());
        }
      });
  }
private:
  geometry_msgs::msg::PointStamped point(double x, double y, double z,
    const geometry_msgs::msg::TransformStamped & transform) {
    geometry_msgs::msg::PointStamped input, output;
    input.point.x = x; input.point.y = y; input.point.z = z;
    tf2::doTransform(input, output, transform);
    return output;
  }
  void publish() {
    auto message = map_.to_occupancy_grid();
    message.header.stamp = now();
    publisher_->publish(message);
  }
  Costmap2D map_{10, 10, 0.05};
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleLayerProcessor>());
  rclcpp::shutdown();
}
