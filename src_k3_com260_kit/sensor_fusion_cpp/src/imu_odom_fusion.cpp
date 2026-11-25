#include <cmath>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Planar EKF: x, y, yaw, world-frame vx, vy, angular velocity.
class ImuOdomFusion : public rclcpp::Node {
  using Odom=nav_msgs::msg::Odometry;
  using Imu=sensor_msgs::msg::Imu;
  using Vector=Eigen::Matrix<double,6,1>;
  using Matrix=Eigen::Matrix<double,6,6>;
  using Policy=message_filters::sync_policies::ApproximateTime<Odom,Imu>;
public:
  ImuOdomFusion() : Node("imu_odom_fusion"),
      odom(this,"/lidar/odom",rmw_qos_profile_sensor_data),
      imu(this,"/imu",rmw_qos_profile_sensor_data), sync(Policy(100),odom,imu) {
    output=create_publisher<Odom>("/odometry/filtered",10);
    sync.setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
    sync.registerCallback(&ImuOdomFusion::update,this);
  }
private:
  void update(const Odom::ConstSharedPtr & lidar,const Imu::ConstSharedPtr & inertial) {
    const rclcpp::Time stamp(lidar->header.stamp);
    if(last.nanoseconds() && stamp<=last) {return;}
    const auto & p=lidar->pose.pose; const auto & q=p.orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    const double c=std::cos(yaw),s=std::sin(yaw);
    Vector z;
    z<<p.position.x,p.position.y,yaw,
       c*lidar->twist.twist.linear.x-s*lidar->twist.twist.linear.y,
       s*lidar->twist.twist.linear.x+c*lidar->twist.twist.linear.y,inertial->angular_velocity.z;
    if(last.nanoseconds()) {
      const double dt=(stamp-last).seconds();
      Matrix F=Matrix::Identity(); F(0,3)=dt; F(1,4)=dt; F(2,5)=dt;
      state=F*state; covariance=F*covariance*F.transpose()+Matrix::Identity()*0.01*dt;
    } else {state=z;}
    last=stamp;
    Vector residual=z-state;
    residual[2]=std::atan2(std::sin(residual[2]),std::cos(residual[2]));
    Matrix R=Matrix::Identity()*0.05;
    R(5,5)=inertial->angular_velocity_covariance[8]>0?inertial->angular_velocity_covariance[8]:0.1;
    const Matrix K=covariance*(covariance+R).inverse();
    state+=K*residual;
    const Matrix A=Matrix::Identity()-K;
    covariance=A*covariance*A.transpose()+K*R*K.transpose();
    Odom msg;
    msg.header=lidar->header; msg.child_frame_id="base_link";
    msg.pose.pose.position.x=state[0]; msg.pose.pose.position.y=state[1];
    msg.pose.pose.orientation.z=std::sin(state[2]/2); msg.pose.pose.orientation.w=std::cos(state[2]/2);
    msg.twist.twist.linear.x=std::cos(state[2])*state[3]+std::sin(state[2])*state[4];
    msg.twist.twist.linear.y=-std::sin(state[2])*state[3]+std::cos(state[2])*state[4];
    msg.twist.twist.angular.z=state[5];
    msg.pose.covariance[0]=covariance(0,0); msg.pose.covariance[7]=covariance(1,1); msg.pose.covariance[35]=covariance(2,2);
    output->publish(msg);
  }
  rclcpp::Time last{0,0,RCL_ROS_TIME};
  Vector state=Vector::Zero(); Matrix covariance=Matrix::Identity()*0.1;
  rclcpp::Publisher<Odom>::SharedPtr output;
  message_filters::Subscriber<Odom> odom;
  message_filters::Subscriber<Imu> imu;
  message_filters::Synchronizer<Policy> sync;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<ImuOdomFusion>()); rclcpp::shutdown();
}
