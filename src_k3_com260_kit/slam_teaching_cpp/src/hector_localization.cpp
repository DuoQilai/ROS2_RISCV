#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "slam_teaching_cpp/hector.hpp"
class HectorLocalization : public rclcpp::Node {
public:
  HectorLocalization() : Node("hector_localization") {
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        if (map_.size() != 0) { return; }
        resolution_ = msg->info.resolution;
        origin_ << msg->info.origin.position.x, msg->info.origin.position.y;
        map_.resize(msg->info.height, msg->info.width);
        for (int y = 0; y < map_.rows(); ++y) { for (int x = 0; x < map_.cols(); ++x) {
          const auto value = msg->data.at(y * msg->info.width + x); map_(y, x) = value < 0 ? 0.5 : value / 100.0;
        }}
        RCLCPP_INFO(get_logger(), "已知地图已加载: %ux%u", msg->info.width, msg->info.height);
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        if (map_.size() == 0) { return; }
        slam_teaching::Points points;
        for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
          const double r = msg->ranges[i], angle = msg->angle_min + i * msg->angle_increment;
          if (std::isfinite(r)) { points.emplace_back(r * std::cos(angle), r * std::sin(angle)); }
        }
        current_pose_ = slam_teaching::HectorScanMatcher().match(points, map_, resolution_, current_pose_, origin_);
        RCLCPP_INFO(get_logger(), "激光位姿=(%.3f,%.3f,%.3f)", current_pose_.x(), current_pose_.y(), current_pose_.z());
      });
  }
private:
  slam_teaching::MatrixXd map_; double resolution_ = 0;
  slam_teaching::Vector2d origin_ = slam_teaching::Vector2d::Zero();
  slam_teaching::Vector3d current_pose_ = slam_teaching::Vector3d::Zero();
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<HectorLocalization>()); rclcpp::shutdown();
}
