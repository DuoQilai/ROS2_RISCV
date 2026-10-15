#pragma once
#include "slam_teaching_cpp/hector.hpp"
#include <random>
namespace slam_teaching {
// BEGIN ICPWithAnalysis
class ICPWithAnalysis : public ICP {
public:
  void analyze_convergence(const Points & source,const Points & target) const {
    for (double offset:{0.0,0.5,1.0,2.0}) {
      const auto result=align(source,target,Vector3d(offset,offset,offset*0.1));
      std::cout << "init_offset=" << offset << " final_error=" << result.errors.back()
                << " iterations=" << result.errors.size() << " converged=" << (result.errors.back()<0.01) << '\n';
    }
    std::mt19937 rng(42);std::normal_distribution<double> normal(0,1);
    const Vector2d expected_t=-rotation(0.3).transpose()*Vector2d(1.0,0.5);
    for (double noise:{0.0,0.05,0.1,0.2,0.5}) {
      Points noisy=source;
      for (auto & p:noisy) { p.x()+=normal(rng)*noise;p.y()+=normal(rng)*noise; }
      const auto result=align(noisy,target);
      std::cout << "noise=" << noise << " angle_error=" << std::abs(wrap(result.pose.z()+0.3))
                << " translation_error=" << (result.pose.head<2>()-expected_t).norm() << '\n';
    }
  }
};
// END ICPWithAnalysis

// BEGIN GaussNewtonScanMatcher
class GaussNewtonScanMatcher {
public:
  int max_iterations=10;double convergence_delta=1e-4;
  Vector3d optimize(const Points & points,Vector3d pose,const MatrixXd & map,
    const MatrixXd & gradient_x,const MatrixXd & gradient_y) const {
    for (int step=0;step<max_iterations;++step) {
      Eigen::Matrix3d H=Eigen::Matrix3d::Zero();Vector3d b=Vector3d::Zero();
      Eigen::Matrix2d derivative;
      derivative << -std::sin(pose.z()),-std::cos(pose.z()),std::cos(pose.z()),-std::sin(pose.z());
      for (const auto & point:points) {
        const Vector2d transformed=rotation(pose.z())*point+pose.head<2>();
        const double residual=1.0-HectorScanMatcher::interpolate(map,transformed);
        const Vector2d gradient(HectorScanMatcher::interpolate(gradient_x,transformed,0),
          HectorScanMatcher::interpolate(gradient_y,transformed,0));
        const Vector3d J(-gradient.x(),-gradient.y(),-gradient.dot(derivative*point));
        H+=J*J.transpose();b+=J*residual;
      }
      const Vector3d delta=(H+Eigen::Matrix3d::Identity()*1e-6).ldlt().solve(-b);
      pose+=delta;if (delta.norm()<convergence_delta) { break; }
    }
    return pose;
  }
};
// END GaussNewtonScanMatcher

// BEGIN corridor_matching_example
inline void corridor_matching_example() {
  Points target,source;std::mt19937 rng(42);std::normal_distribution<double> normal(0,0.02);
  for (int i=0;i<200;++i) { const double x=-5.0+10.0*i/199.0;target.emplace_back(x,-1.0);target.emplace_back(x,1.0); }
  for (const auto & p:target) {
    const double nx=normal(rng),ny=normal(rng);
    source.push_back(rotation(0.05)*p+Vector2d(0.5,0.0)+Vector2d(nx,ny));
  }
  const auto icp=ICP(30).align(source,target);const auto plicp=PointToLineICP(15).align(source,target);
  std::cout << "走廊 ICP误差=" << icp.errors.back() << " 迭代=" << icp.errors.size() << '\n'
            << "走廊 PLICP误差=" << plicp.errors.back() << " 迭代=" << plicp.errors.size() << '\n';
}
// END corridor_matching_example

// BEGIN RobustScanMatcher
class RobustScanMatcher {
public:
  int max_iterations=30;
  static double huber_weight(double residual,double delta=1.0) { return residual>delta ? delta/residual : 1.0; }
  Vector3d align(const Points & source,const Points & target,Vector3d pose=Vector3d::Zero()) const {
    if (source.empty() || target.empty()) { throw std::invalid_argument("鲁棒ICP要求非空点云"); }
    for (int step=0;step<max_iterations;++step) {
      Points transformed,matched;std::vector<double> weights;double total=0;
      Vector2d mu_source=Vector2d::Zero(),mu_target=Vector2d::Zero();
      for (const auto & p:source) {
        Vector2d current=rotation(pose.z())*p+pose.head<2>(),q=target[nearest(current,target)];
        const double weight=huber_weight((current-q).norm(),0.5);
        transformed.push_back(current);matched.push_back(q);weights.push_back(weight);
        mu_source+=weight*current;mu_target+=weight*q;total+=weight;
      }
      mu_source/=total;mu_target/=total;Eigen::Matrix2d H=Eigen::Matrix2d::Zero();
      for (std::size_t i=0;i<source.size();++i) {
        H+=weights[i]*(transformed[i]-mu_source)*(matched[i]-mu_target).transpose();
      }
      Eigen::JacobiSVD<Eigen::Matrix2d> svd(H,Eigen::ComputeFullU|Eigen::ComputeFullV);
      Eigen::Matrix2d V=svd.matrixV(),R=V*svd.matrixU().transpose();
      if (R.determinant()<0) { V.col(1)*=-1;R=V*svd.matrixU().transpose(); }
      pose.head<2>()=R*pose.head<2>()+mu_target-R*mu_source;
      pose.z()=wrap(pose.z()+std::atan2(R(1,0),R(0,0)));
    }
    return pose;
  }
};
// END RobustScanMatcher

// BEGIN analyze_scan_matching
inline void analyze_scan_matching(const MatrixXd & map,const Points & scan) {
  const std::vector<Vector3d> offsets{Vector3d(0,0,0),Vector3d(0.1,0.1,0.05),Vector3d(0.5,0.5,0.2),Vector3d(1,1,0.5)};
  for (const auto & offset:offsets) {
    const auto optimized=HectorScanMatcher().match(scan,map,0.05,Vector3d(5,5,0)+offset);
    const double error=(optimized-Vector3d(5,5,0)).norm();
    std::cout << "offset=" << offset.transpose() << " final_error=" << error << " converged=" << (error<0.1) << '\n';
  }
}
// END analyze_scan_matching

// BEGIN generate_mapping_path
inline std::string generate_mapping_path(const std::string & environment) {
  const std::map<std::string,std::string> strategies{
    {"office",R"(办公环境建图策略:
1. 沿走廊中轴线前进，保证两侧墙壁被扫描到
2. 在每个房间门口做“回”字形扫描
3. 避免急转弯，保持匀速
4. 完成所有房间后，沿外环走一圈巩固闭环)"},
    {"warehouse",R"(仓库环境建图策略:
1. 沿货架通道穿行，逐行扫描
2. 每个通道两端做180°旋转覆盖
3. 注意货架间的狭窄通道
4. 保持货架两侧都有扫描覆盖)"},
    {"large_hall",R"(大展厅建图策略:
1. 沿墙壁走一圈建立基本轮廓
2. 使用“S”形路径覆盖内部空间
3. 在空旷区域适当提高移动速度
4. 回到起点的闭环至关重要)"}
  };
  const auto it=strategies.find(environment);return it==strategies.end() ? "通用策略" : it->second;
}
// END generate_mapping_path
}  // namespace slam_teaching
