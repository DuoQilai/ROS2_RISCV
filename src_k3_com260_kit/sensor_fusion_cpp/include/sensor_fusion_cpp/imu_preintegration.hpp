#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct IMUState {
  Eigen::Vector3d position=Eigen::Vector3d::Zero(), velocity=Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();
};

class IMUPreintegrator {
public:
  void integrate(const Eigen::Vector3d & acc, const Eigen::Vector3d & gyro) {
    const Eigen::Vector3d acceleration=delta.rotation*(acc-acc_bias);
    delta.position+=delta.velocity*dt+0.5*acceleration*dt*dt;
    delta.velocity+=acceleration*dt;
    const Eigen::Vector3d angle=(gyro-gyro_bias)*dt;
    if (angle.norm()>1e-12) {
      delta.rotation*=Eigen::AngleAxisd(angle.norm(),angle.normalized()).toRotationMatrix();
    }
    total_time+=dt;
  }
  IMUState predict(const IMUState & initial) const {
    const Eigen::Vector3d gravity(0,0,-9.81);
    return {initial.position+initial.velocity*total_time+initial.rotation*delta.position
              +0.5*gravity*total_time*total_time,
            initial.velocity+initial.rotation*delta.velocity+gravity*total_time,
            initial.rotation*delta.rotation};
  }
  double dt=0.01, total_time=0;
  Eigen::Vector3d gyro_bias=Eigen::Vector3d::Zero(), acc_bias=Eigen::Vector3d::Zero();
  IMUState delta;
};
