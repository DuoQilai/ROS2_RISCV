#pragma once
#include "slam_teaching_cpp/hector.hpp"
#include <deque>
#include <optional>
namespace slam_teaching {
// BEGIN PoseGraphOptimization
class PoseGraphOptimization {
public:
  struct Edge { int i,j; Vector3d measurement; Eigen::Matrix3d information; };
  std::vector<Vector3d> nodes; std::vector<Edge> edges;
  int add_node(const Vector3d & pose) { nodes.push_back(pose); return static_cast<int>(nodes.size())-1; }
  void add_edge(int i,int j,const Vector3d & measurement,const Eigen::Matrix3d & info) { edges.push_back({i,j,measurement,info}); }
  static Vector3d relative(const Vector3d & a,const Vector3d & b) {
    Vector3d result; result << rotation(a.z()).transpose()*(b.head<2>()-a.head<2>()),wrap(b.z()-a.z()); return result;
  }
  static Vector3d edge_error(const Vector3d & a,const Vector3d & b,const Vector3d & measurement) {
    Vector3d error=measurement-relative(a,b); error.z()=wrap(error.z()); return error;
  }
  double compute_error() const {
    double total=0; for (const auto & edge:edges) {
      Vector3d e=edge_error(nodes.at(edge.i),nodes.at(edge.j),edge.measurement); total+=e.dot(edge.information*e);
    } return total;
  }
  static std::pair<Eigen::Matrix3d,Eigen::Matrix3d> jacobians(const Vector3d & a,const Vector3d & b) {
    Eigen::Matrix3d Ji=Eigen::Matrix3d::Zero(),Jj=Eigen::Matrix3d::Zero();
    Vector3d predicted=relative(a,b);
    Ji.topLeftCorner<2,2>()=rotation(a.z()).transpose(); Ji(0,2)=-predicted.y(); Ji(1,2)=predicted.x(); Ji(2,2)=1;
    Jj.topLeftCorner<2,2>()=-rotation(a.z()).transpose(); Jj(2,2)=-1;
    return {Ji,Jj};
  }
  std::vector<Vector3d> optimize(int iterations=10) {
    const int n=static_cast<int>(3*nodes.size()); if (n==0) { return nodes; }
    for (int step=0;step<iterations;++step) {
      MatrixXd H=MatrixXd::Zero(n,n); VectorXd b=VectorXd::Zero(n);
      for (const auto & edge:edges) {
        const auto [Ji,Jj]=jacobians(nodes.at(edge.i),nodes.at(edge.j));
        Vector3d error=edge_error(nodes[edge.i],nodes[edge.j],edge.measurement); int i=3*edge.i,j=3*edge.j;
        H.block<3,3>(i,i)+=Ji.transpose()*edge.information*Ji; H.block<3,3>(i,j)+=Ji.transpose()*edge.information*Jj;
        H.block<3,3>(j,i)+=Jj.transpose()*edge.information*Ji; H.block<3,3>(j,j)+=Jj.transpose()*edge.information*Jj;
        b.segment<3>(i)+=Ji.transpose()*edge.information*error; b.segment<3>(j)+=Jj.transpose()*edge.information*error;
      }
      H.topRows(3).setZero(); H.leftCols(3).setZero(); H.topLeftCorner<3,3>().setIdentity(); b.head<3>().setZero();
      VectorXd delta=H.ldlt().solve(-b);
      if (!delta.allFinite()) { throw std::runtime_error("位姿图不可解，请检查图连通性和约束"); }
      for (std::size_t i=0;i<nodes.size();++i) { nodes[i]+=delta.segment<3>(3*i); nodes[i].z()=wrap(nodes[i].z()); }
      if (delta.norm()<1e-4) { break; }
    } return nodes;
  }
};
// END PoseGraphOptimization

// BEGIN LocalSLAMComponents
class VoxelFilter {
public:
  explicit VoxelFilter(double size) : voxel_size(size) {}
  double voxel_size;
  Points filter(const Points & points) const {
    std::map<std::pair<int,int>,Vector2d> voxels;
    for (const auto & p:points) { voxels.emplace(std::make_pair(static_cast<int>(std::floor(p.x()/voxel_size)),
      static_cast<int>(std::floor(p.y()/voxel_size))),p); }
    Points output; for (const auto & item:voxels) { output.push_back(item.second); } return output;
  }
};
class PoseExtrapolator {
public:
  std::deque<Vector3d> poses; std::deque<double> timestamps; Vector3d velocity=Vector3d::Zero();
  void add_pose(const Vector3d & pose,double time) {
    if (!poses.empty() && time>timestamps.back()) { velocity=(pose-poses.back())/(time-timestamps.back()); }
    poses.push_back(pose); timestamps.push_back(time);
    if (poses.size()>10) { poses.pop_front(); timestamps.pop_front(); }
  }
  Vector3d extrapolate(const Vector3d & odom_pose,double timestamp=0.0) const {
    (void)timestamp;  // 原片段固定外推0.1s；不是基于传入时间的真实同步器。
    return poses.empty() ? odom_pose : Vector3d(poses.back()+velocity*0.1);
  }
};
class MotionFilter {
public:
  double max_distance=0.3,max_angle=std::acos(-1.0)/6,max_time=1.0;
  std::optional<Vector3d> last_pose; double last_time=0;
  bool should_insert(const Vector3d & pose,double time) {
    if (!last_pose || (pose.head<2>()-last_pose->head<2>()).norm()>max_distance ||
      std::abs(wrap(pose.z()-last_pose->z()))>max_angle || time-last_time>max_time) {
      last_pose=pose; last_time=time; return true;
    } return false;
  }
};
class Submap {
public:
  double resolution; Vector3d origin; MatrixXd grid=MatrixXd::Zero(200,200); int scans=0,max_scans=50;
  Submap(double res,const Vector3d & start) : resolution(res),origin(start) {}
  void insert_scan(const Points & points,const Vector3d & pose) {
    ++scans;
    for (const auto & p:points) {
      Vector2d cell=(rotation(pose.z())*p+pose.head<2>()-origin.head<2>())/resolution;
      int x=static_cast<int>(cell.x()),y=static_cast<int>(cell.y());
      if (x>=0 && y>=0 && x<200 && y<200) { grid(y,x)=std::min(grid(y,x)+0.5,5.0); }
    }
  }
  bool is_complete() const { return scans>=max_scans; }
  std::pair<double,Vector2d> bilinear_interpolate(const Vector2d & p) const {
    int x=static_cast<int>(std::floor(p.x())),y=static_cast<int>(std::floor(p.y()));
    if (x<0 || y<0 || x>=199 || y>=199) { return {0.5,Vector2d::Zero()}; }
    double dx=p.x()-x,dy=p.y()-y;
    const double log_odds=HectorScanMatcher::interpolate(grid,p,0.0),prob=1.0/(1.0+std::exp(-log_odds));
    Vector2d gradient(((grid(y,x+1)-grid(y,x))*(1-dy)+(grid(y+1,x+1)-grid(y+1,x))*dy)/resolution,
      ((grid(y+1,x)-grid(y,x))*(1-dx)+(grid(y+1,x+1)-grid(y,x+1))*dx)/resolution);
    return {prob,gradient*prob*(1-prob)};
  }
};
class LocalSLAM {
public:
  double resolution=0.05; std::optional<Submap> current_submap;
  PoseExtrapolator extrapolator; VoxelFilter voxel{0.05}; MotionFilter motion;
  std::optional<Vector3d> process_scan(const Points & scan,const Vector3d & odom_pose,double timestamp=0.0) {
    Points filtered=voxel.filter(scan); Vector3d pose=extrapolator.extrapolate(odom_pose,timestamp);
    if (current_submap) {
      for (int step=0;step<10;++step) {
        Eigen::Matrix3d H=Eigen::Matrix3d::Zero(); Vector3d b=Vector3d::Zero();
        Eigen::Matrix2d dR; dR << -std::sin(pose.z()),-std::cos(pose.z()),std::cos(pose.z()),-std::sin(pose.z());
        for (const auto & point:filtered) {
          Vector2d cell=(rotation(pose.z())*point+pose.head<2>()-current_submap->origin.head<2>())/resolution;
          const auto [prob,gradient]=current_submap->bilinear_interpolate(cell);
          Vector3d J(-gradient.x(),-gradient.y(),-gradient.dot(dR*point));
          H+=J*J.transpose(); b+=J*(1-prob);
        }
        Vector3d delta=(H+1e-6*Eigen::Matrix3d::Identity()).ldlt().solve(-b); pose+=delta;
        if (delta.norm()<1e-4) { break; }
      }
    }
    if (!motion.should_insert(pose,timestamp)) { return std::nullopt; }
    if (!current_submap) { current_submap.emplace(resolution,pose); }
    current_submap->insert_scan(filtered,pose); extrapolator.add_pose(pose,timestamp);
    if (current_submap->is_complete()) { current_submap.reset(); }
    return pose;
  }
};
// END LocalSLAMComponents

// BEGIN FastCorrelativeScanMatcher
class FastCorrelativeScanMatcher {
public:
  const Submap & submap;
  explicit FastCorrelativeScanMatcher(const Submap & map) : submap(map) {}
  double compute_score(const Points & scan,const Vector3d & pose) const {
    if (scan.empty()) { throw std::invalid_argument("相关匹配需要非空扫描"); }
    double score=0;
    for (const auto & point:scan) {
      Vector2d cell=(rotation(pose.z())*point+pose.head<2>()-submap.origin.head<2>())/submap.resolution;
      int x=static_cast<int>(cell.x()),y=static_cast<int>(cell.y());
      if (x>=0 && y>=0 && x<200 && y<200 && submap.grid(y,x)>0) { score+=1; }
    } return score/scan.size();
  }
  std::pair<Vector3d,double> match(const Points & scan,const Vector3d & initial,
    const Vector3d & window=Vector3d(3.0,3.0,0.5)) const {
    Vector3d best=initial; double score=-std::numeric_limits<double>::infinity();
    for (int a=0;a<static_cast<int>(window.z()/0.01);++a) {
      for (int x=0;x<static_cast<int>(window.x()/0.05);x+=5) {
        for (int y=0;y<static_cast<int>(window.y()/0.05);y+=5) {
          Vector3d candidate=initial-window/2+Vector3d(x*0.05,y*0.05,a*0.01);
          double value=compute_score(scan,candidate); if (value>score) { score=value; best=candidate; }
        }
      }
    }
    if (score>0.5) {
      const Vector3d center=best;
      for (int x=-5;x<=5;++x) { for (int y=-5;y<=5;++y) {
        Vector3d candidate=center+Vector3d(x*0.01,y*0.01,0); double value=compute_score(scan,candidate);
        if (value>score) { score=value; best=candidate; }
      }}
    }
    return {best,score};
  }
};
// END FastCorrelativeScanMatcher
}  // namespace slam_teaching
