#pragma once
#include <Eigen/Core>
#include <string>
#include <vector>

class VINSMono {
public:
  struct State {
    Eigen::Vector3d position = Eigen::Vector3d::Zero(), velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero(), acc_bias = Eigen::Vector3d::Zero();
    Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
  } state;
  struct Sample { Eigen::Vector3d acc, gyro; double dt; State state; };
  std::vector<Sample> imu_buffer;
  double gyro_noise = 0.01, acc_noise = 0.01, gyro_bias_noise = 0.0001, acc_bias_noise = 0.0001;
  void process_imu(const Eigen::Vector3d & acc, const Eigen::Vector3d & gyro, double dt) {
    const Eigen::Vector3d a = acc - state.acc_bias, w = gyro - state.gyro_bias;
    state.position += state.velocity * dt + 0.5 * a * dt * dt;
    state.velocity += a * dt;
    Eigen::Matrix3d omega;
    omega << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
    state.orientation *= Eigen::Matrix3d::Identity() + omega * dt;
    imu_buffer.push_back({acc, gyro, dt, state});
  }
  // The original lesson leaves visual matching, alignment and sliding-window optimization as exercises.
};

inline std::string suggest_fusion_strategy(const std::string &, bool lidar, bool camera, bool imu,
                                         const std::string & budget) {
  if (budget == "low") { return camera && imu ? "VINS-Mono (仅视觉+IMU)" : "ORB-SLAM2 (仅视觉)"; }
  if (budget == "medium") { return lidar && camera ? "LVI-SAM / Cartographer+视觉回环" : "Cartographer (激光+IMU)"; }
  if (budget == "high") { return lidar && camera && imu ? "R3LIVE / LVI-SAM (全传感器融合)" : "FAST-LIO2 (激光+IMU)"; }
  return "推荐使用激光SLAM建图 + 视觉SLAM提供语义信息";
}
