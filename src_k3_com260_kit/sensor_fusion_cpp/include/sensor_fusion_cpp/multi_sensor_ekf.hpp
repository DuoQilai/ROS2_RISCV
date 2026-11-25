#pragma once
#include <Eigen/Dense>

class MultiSensorEKF {
public:
  using Vector=Eigen::Matrix<double,15,1>;
  using Matrix=Eigen::Matrix<double,15,15>;
  Vector x=Vector::Zero();
  Matrix P=Matrix::Identity()*0.1, Q=Matrix::Identity()*0.01;
  void predict(double dt) {
    Matrix F=Matrix::Identity();
    F.block<3,3>(0,3)=Eigen::Matrix3d::Identity()*dt;
    x=F*x; P=F*P*F.transpose()+Q;
  }
  void update(const Eigen::VectorXd & z, const Eigen::MatrixXd & H, double variance) {
    const Eigen::MatrixXd R=Eigen::MatrixXd::Identity(z.size(),z.size())*variance;
    const Eigen::MatrixXd K=P*H.transpose()*(H*P*H.transpose()+R).inverse();
    x+=K*(z-H*x);
    const Matrix A=Matrix::Identity()-K*H;
    P=A*P*A.transpose()+K*R*K.transpose();
  }
  void update_lidar_odom(const Eigen::Matrix<double,6,1> & z) {
    update(z,Eigen::Matrix<double,6,15>::Identity(),0.05);
  }
  void update_visual_odom(const Eigen::Matrix<double,6,1> & z) {
    update(z,Eigen::Matrix<double,6,15>::Identity(),0.2);
  }
  void update_gps(const Eigen::Vector3d & position) {
    update(position,Eigen::Matrix<double,3,15>::Identity(),1.0);
  }
  void update_imu(const Eigen::Matrix<double,6,1> & z,
                  const Eigen::Matrix<double,6,15> & measurement_jacobian) {
    update(z,measurement_jacobian,0.1);
  }
};
