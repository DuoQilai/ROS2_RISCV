#include <iomanip>
#include <iostream>
#include "slam_teaching_cpp/cartographer_concepts.hpp"
int main() {
  using namespace slam_teaching;
  PoseGraphOptimization graph; graph.add_node(Vector3d::Zero()); graph.add_node(Vector3d(1.2,0.1,0.1));
  graph.add_node(Vector3d(2.2,-0.1,-0.1));
  graph.add_edge(0,1,Vector3d(1,0,0),Eigen::Matrix3d::Identity()*100);
  graph.add_edge(1,2,Vector3d(1,0,0),Eigen::Matrix3d::Identity()*100);
  graph.add_edge(0,2,Vector3d(2,0,0),Eigen::Matrix3d::Identity()*50);
  double before=graph.compute_error(); graph.optimize(20); double after=graph.compute_error();
  const Vector3d a(0.3,-0.2,0.4),b(1.1,0.7,-0.3),measurement(0.8,0.2,-0.7);
  const auto [Ji,Jj]=PoseGraphOptimization::jacobians(a,b); Eigen::Matrix3d numeric_i,numeric_j;
  constexpr double eps=1e-6;
  for (int k=0;k<3;++k) {
    Vector3d delta=Vector3d::Zero(); delta[k]=eps;
    numeric_i.col(k)=(PoseGraphOptimization::edge_error(a+delta,b,measurement)-PoseGraphOptimization::edge_error(a-delta,b,measurement))/(2*eps);
    numeric_j.col(k)=(PoseGraphOptimization::edge_error(a,b+delta,measurement)-PoseGraphOptimization::edge_error(a,b-delta,measurement))/(2*eps);
  }
  double jacobian_error=std::max((Ji-numeric_i).cwiseAbs().maxCoeff(),(Jj-numeric_j).cwiseAbs().maxCoeff());
  VoxelFilter voxel(0.05); auto filtered=voxel.filter({Vector2d(0,0),Vector2d(0.01,0.01),Vector2d(0.06,0)});
  MotionFilter motion; bool first=motion.should_insert(Vector3d::Zero(),0),still=motion.should_insert(Vector3d::Zero(),0.5),moved=motion.should_insert(Vector3d(0.31,0,0),0.6);
  PoseExtrapolator predictor; predictor.add_pose(Vector3d::Zero(),0); predictor.add_pose(Vector3d(1,0,0.1),1);
  const auto predicted=predictor.extrapolate(Vector3d::Zero(),1.1);
  std::cout << std::fixed << std::setprecision(9) << "graph before=" << before << " after=" << after
    << " pose2=" << graph.nodes[2].transpose() << " jacobian_max_error=" << jacobian_error << '\n';
  std::cout << "voxel_count=" << filtered.size() << " motion=" << first << ',' << still << ',' << moved
    << " extrapolated=" << predicted.transpose() << '\n';
  bool pass=after<1e-12 && jacobian_error<1e-7 && filtered.size()==2 && first && !still && moved &&
    (predicted-Vector3d(1.1,0,0.11)).norm()<1e-12;
  std::cout << "pass=" << std::boolalpha << pass << '\n'; return pass ? 0 : 1;
}
