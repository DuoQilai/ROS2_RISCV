#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "slam_teaching_cpp/scan_matching.hpp"
class ScanMatcherNode : public rclcpp::Node {
public:
  ScanMatcherNode() : Node("scan_matcher") {
    scan_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        latest_.clear();
        for (std::size_t i=0;i<msg->ranges.size();++i) {
          double r=msg->ranges[i],a=msg->angle_min+i*msg->angle_increment;
          if (std::isfinite(r) && r>msg->range_min && r<msg->range_max) { latest_.emplace_back(r*std::cos(a),r*std::sin(a)); }
        }
        if (++scan_count_>5 && reference_.empty() && latest_.size()>=6) {
          reference_=latest_; reference_odom_=odom_pose_;
          RCLCPP_INFO(get_logger(),"参考扫描已初始化: %zu点",reference_.size());
        }
      });
    odom_=create_subscription<nav_msgs::msg::Odometry>("/odom",10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & p=msg->pose.pose; const auto & q=p.orientation;
        odom_pose_ << p.position.x,p.position.y,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      });
    timer_=create_wall_timer(std::chrono::milliseconds(500),[this](){
      if (latest_.size()<6 || reference_.size()<6) { return; }
      slam_teaching::Vector3d initial;
      initial << slam_teaching::rotation(reference_odom_.z()).transpose()*(odom_pose_.head<2>()-reference_odom_.head<2>()),
        slam_teaching::wrap(odom_pose_.z()-reference_odom_.z());
      const auto result=slam_teaching::PointToLineICP().align(latest_,reference_,initial);
      RCLCPP_INFO(get_logger(),"扫描匹配: 位姿=(%.3f, %.3f, %.1f°) 误差=%.6f 迭代=%zu",
        result.pose.x(),result.pose.y(),result.pose.z()*180/std::acos(-1.0),result.errors.back(),result.errors.size());
      if (result.errors.back()<0.01) { reference_=latest_; reference_odom_=odom_pose_; }
    });
  }
private:
  slam_teaching::Points latest_,reference_; std::size_t scan_count_=0;
  slam_teaching::Vector3d odom_pose_=slam_teaching::Vector3d::Zero(),reference_odom_=slam_teaching::Vector3d::Zero();
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<ScanMatcherNode>()); rclcpp::shutdown();
}
