#include <cmath>
#include <iostream>
#include <stdexcept>
#include "sensor_fusion_cpp/imu_preintegration.hpp"
#include "sensor_fusion_cpp/multi_sensor_ekf.hpp"
#include "sensor_fusion_cpp/sensor_health.hpp"
#include "sensor_fusion_cpp/lidar_features.hpp"

int main() {
  auto require=[](bool ok,const char * name) {
    if(!ok) {throw std::runtime_error(name);}
    std::cout<<name<<": PASS\n";
  };
  IMUPreintegrator stationary,accelerating,turning;
  for(int i=0;i<100;++i) {
    stationary.integrate({0,0,9.81},{0,0,0});
    accelerating.integrate({1,0,9.81},{0,0,0});
    turning.integrate({0,0,9.81},{0,0,M_PI/2});
  }
  const auto still=stationary.predict(IMUState{}), moved=accelerating.predict(IMUState{});
  require(still.position.norm()<1e-9 && still.velocity.norm()<1e-9,"静止 IMU 重力抵消");
  require((moved.position-Eigen::Vector3d(.5,0,0)).norm()<1e-9
      && (moved.velocity-Eigen::Vector3d(1,0,0)).norm()<1e-9,"1 秒恒加速度积分");
  const Eigen::Vector3d rotated=turning.predict(IMUState{}).rotation*Eigen::Vector3d::UnitX();
  require((rotated-Eigen::Vector3d::UnitY()).norm()<1e-9,"1 秒 90 度旋转");
  MultiSensorEKF filter;
  filter.update_gps({1,0,0});
  require(std::abs(filter.x[0]-1.0/11)<1e-9,"GPS Kalman 增益");
  filter.update_lidar_odom(Eigen::Matrix<double,6,1>::Ones());
  require(filter.P.isApprox(filter.P.transpose()) && filter.P.llt().info()==Eigen::Success,"融合协方差正定");
  SensorHealthMonitor health;
  health.received("imu",1.0);
  require(health.check_health("imu",1.19) && !health.check_health("imu",1.21),"IMU 超时检测");
  health.received("imu",1.3);
  require(health.check_health("imu",1.31),"IMU 恢复检测");
  PointCloud points;
  for (int i=0;i<21;++i) { points.push_back({double(i),i==10?5.0:0.0,0.0}); }
  const auto features=extract_features(points);
  require(features.edge.size()==4 && features.flat.size()==4,"曲率分位数特征提取");
  require(extract_features({}).edge.empty(),"空点云特征提取");
}
