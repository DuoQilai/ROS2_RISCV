#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
class AMCLMonitor : public rclcpp::Node {
public:
  AMCLMonitor() : Node("amcl_monitor") {
    cloud_=create_subscription<nav2_msgs::msg::ParticleCloud>("/particle_cloud",rclcpp::SensorDataQoS(),
      [this](nav2_msgs::msg::ParticleCloud::ConstSharedPtr msg) {
        if (msg->particles.empty()) { return; }
        double x=0,y=0; for (const auto & p:msg->particles) { x+=p.pose.position.x;y+=p.pose.position.y; }
        x/=msg->particles.size();y/=msg->particles.size();double vx=0,vy=0;
        for (const auto & p:msg->particles) { vx+=std::pow(p.pose.position.x-x,2);vy+=std::pow(p.pose.position.y-y,2); }
        spreads_.push_back({std::sqrt(vx/msg->particles.size()),std::sqrt(vy/msg->particles.size())});
        if (spreads_.size()>100) { spreads_.pop_front(); }
      });
    pose_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose",10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        positions_.push_back({msg->pose.pose.position.x,msg->pose.pose.position.y});
        if (positions_.size()>100) { positions_.pop_front(); }
        covariance_=msg->pose.covariance[0]+msg->pose.covariance[7]+msg->pose.covariance[14];
      });
    timer_=create_wall_timer(std::chrono::seconds(1),[this](){
      if (positions_.empty() || spreads_.empty()) { return; }
      double sx=0,sy=0,x=0,y=0;
      for (const auto & p:spreads_) { sx+=p.first;sy+=p.second; }
      for (const auto & p:positions_) { x+=p.first;y+=p.second; }
      x/=positions_.size();y/=positions_.size();double vx=0,vy=0;
      for (const auto & p:positions_) { vx+=std::pow(p.first-x,2);vy+=std::pow(p.second-y,2); }
      RCLCPP_INFO(get_logger(),"cov_trace=%.6f avg_particle_spread=(%.6f,%.6f) position_variance=(%.6f,%.6f)",
        covariance_,sx/spreads_.size(),sy/spreads_.size(),vx/positions_.size(),vy/positions_.size());
    });
  }
private:
  std::deque<std::pair<double,double>> positions_,spreads_;double covariance_=0;
  rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr cloud_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<AMCLMonitor>());rclcpp::shutdown();
}
