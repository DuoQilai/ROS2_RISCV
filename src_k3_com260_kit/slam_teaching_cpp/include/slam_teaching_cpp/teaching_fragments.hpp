#pragma once
#include "slam_teaching_cpp/localization.hpp"
namespace slam_teaching {
// BEGIN MultiSensorAMCL
inline void apply_imu_heading(std::vector<MCLParticle> & particles,double imu_yaw) {
  for (auto & particle:particles) {
    double difference=std::abs(wrap(particle.pose.z()-imu_yaw));
    const double likelihood=std::exp(-difference*difference/(2*0.05*0.05));
    particle.weight*=likelihood;
  }
}
// END MultiSensorAMCL

// BEGIN LocalizationHealthMonitor
struct LocalizationHealthMonitor {
  int consecutive_bad_readings=0;bool auto_recovery_enabled=true;
  bool needs_recovery(const std::vector<Vector2d> & positions) {
    if (positions.size()<10) { return false; }
    Vector2d mean=Vector2d::Zero();for (const auto & p:positions) { mean+=p; }mean/=positions.size();
    Vector2d variance=Vector2d::Zero();
    for (const auto & p:positions) { variance+=(p-mean).array().square().matrix(); }
    const double spread=(variance/positions.size()).array().sqrt().mean();
    if (spread>2.0) { ++consecutive_bad_readings; }
    else { consecutive_bad_readings=0; }
    return consecutive_bad_readings>10 && auto_recovery_enabled;
  }
};
// END LocalizationHealthMonitor

// BEGIN CartographerCore
// 流程声明：依赖尚未定义的历史子图锚点与约束生成规则，不能实例化为建图节点。
struct CartographerCore {
  int submap_size=10;
  // 1. 根据里程计、上一时间戳及历史位姿预测初值。
  // 2. 初值附近以 dx/dy∈[-0.1,0.1]、dtheta∈[-0.05,0.05] 各5档扫描匹配。
  // 3. 保存当前位姿，把扫描终点插入100×100、0.05 m子图。
  // 4. 生成相邻节点的相对位姿约束，信息矩阵100 I。
  // 5. 每10帧新建子图；历史子图匹配得分>0.7时考虑回环。
  // 原例未定义以下两项；须先定义其坐标系、锚点和输入输出，再补全实现。
  Vector3d compute_relative_transform(const Vector3d & previous,const Vector3d & current);
  Vector3d fast_correlative_scan_match(const Points & scan,const Vector3d & initial,int historical_submap);
};
// END CartographerCore
}  // namespace slam_teaching
