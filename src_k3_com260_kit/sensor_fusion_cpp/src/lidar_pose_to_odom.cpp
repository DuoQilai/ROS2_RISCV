#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class LidarPoseToOdom : public rclcpp::Node {
public:
  LidarPoseToOdom() : Node("lidar_pose_to_odom") {
    output=create_publisher<nav_msgs::msg::Odometry>("/lidar/odom",10);
    input=create_subscription<geometry_msgs::msg::PoseStamped>("/tracked_pose",10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (previous) {
          const double dt=(rclcpp::Time(msg->header.stamp)-rclcpp::Time(previous->header.stamp)).seconds();
          if (dt<=0) { return; }
          nav_msgs::msg::Odometry odom;
          odom.header=msg->header; odom.child_frame_id="base_link"; odom.pose.pose=msg->pose;
          const double angle=yaw(msg->pose.orientation);
          const double vx=(msg->pose.position.x-previous->pose.position.x)/dt;
          const double vy=(msg->pose.position.y-previous->pose.position.y)/dt;
          odom.twist.twist.linear.x=std::cos(angle)*vx+std::sin(angle)*vy;
          odom.twist.twist.linear.y=-std::sin(angle)*vx+std::cos(angle)*vy;
          const double delta=angle-yaw(previous->pose.orientation);
          odom.twist.twist.angular.z=std::atan2(std::sin(delta),std::cos(delta))/dt;
          odom.pose.covariance[0]=odom.pose.covariance[7]=odom.pose.covariance[35]=0.05;
          output->publish(odom);
        }
        previous=msg;
      });
  }
private:
  static double yaw(const geometry_msgs::msg::Quaternion & q) {
    return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  }
  geometry_msgs::msg::PoseStamped::ConstSharedPtr previous;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr input;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<LidarPoseToOdom>()); rclcpp::shutdown();
}
