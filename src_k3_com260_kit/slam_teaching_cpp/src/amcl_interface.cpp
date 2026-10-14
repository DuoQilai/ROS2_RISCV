#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
class AMCLInterface : public rclcpp::Node {
public:
  AMCLInterface() : Node("amcl_interface") {
    initial_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10);
    pose_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose",10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        current_=*msg; const auto & p=msg->pose.pose; const auto & q=p.orientation;
        double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"AMCL位姿: (%.3f, %.3f, %.1f°) 不确定度=%.4f",
          p.position.x,p.position.y,yaw*180/std::acos(-1.0),std::sqrt(msg->pose.covariance[0]+msg->pose.covariance[7]));
      });
    particles_=create_subscription<nav2_msgs::msg::ParticleCloud>("/particle_cloud",rclcpp::SensorDataQoS(),
      [this](nav2_msgs::msg::ParticleCloud::ConstSharedPtr msg) { cloud_=*msg; });
    timer_=create_wall_timer(std::chrono::seconds(1),[this](){
      if (requested_ && initial_->get_subscription_count()>0) {
        initial_->publish(*requested_); requested_.reset(); RCLCPP_INFO(get_logger(),"初始位姿已发布");
      }
      if (!current_) { RCLCPP_INFO(get_logger(),"AMCL未初始化"); return; }
      const double quality=get_localization_quality();
      RCLCPP_INFO(get_logger(),"%s (质量=%.2f)",quality>0.7 ? "定位良好" : quality>0.3 ? "定位一般" : "定位差，建议重新初始化",quality);
    });
  }
  void set_initial_pose(double x,double y,double yaw) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id="map"; msg.header.stamp=now(); msg.pose.pose.position.x=x; msg.pose.pose.position.y=y;
    msg.pose.pose.orientation.z=std::sin(yaw/2); msg.pose.pose.orientation.w=std::cos(yaw/2);
    msg.pose.covariance[0]=0.25; msg.pose.covariance[7]=0.25; msg.pose.covariance[35]=0.068;
    requested_=msg;
  }
  double get_localization_quality() const {
    if (!current_ || cloud_.particles.size()<10) { return 0; }
    double x=0,y=0; for (const auto & p:cloud_.particles) { x+=p.pose.position.x; y+=p.pose.position.y; }
    x/=cloud_.particles.size(); y/=cloud_.particles.size(); double vx=0,vy=0;
    for (const auto & p:cloud_.particles) {
      vx+=(p.pose.position.x-x)*(p.pose.position.x-x); vy+=(p.pose.position.y-y)*(p.pose.position.y-y);
    }
    double spread=(std::sqrt(vx/cloud_.particles.size())+std::sqrt(vy/cloud_.particles.size()))/2;
    return std::clamp(1.0/(1.0+(current_->pose.covariance[0]+current_->pose.covariance[7])*10+spread*5),0.0,1.0);
  }
private:
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> current_,requested_;
  nav2_msgs::msg::ParticleCloud cloud_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_;
  rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr particles_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv); auto args=rclcpp::remove_ros_arguments(argc,argv);
  auto node=std::make_shared<AMCLInterface>();
  try {
    if (args.size()==4) {
      double values[3];
      for (std::size_t i=0;i<3;++i) {
        std::size_t used=0; values[i]=std::stod(args[i+1],&used);
        if (used!=args[i+1].size() || !std::isfinite(values[i])) { throw std::invalid_argument("位姿必须为有限数值"); }
      }
      node->set_initial_pose(values[0],values[1],values[2]);
    } else if (args.size()!=1) { throw std::invalid_argument("用法: amcl_interface [x y yaw]"); }
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(),"%s",error.what());rclcpp::shutdown();return 2;
  }
  rclcpp::shutdown();
}
