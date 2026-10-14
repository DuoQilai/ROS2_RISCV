#include <iomanip>
#include <iostream>
#include "slam_teaching_cpp/analysis_examples.hpp"
#include "slam_teaching_cpp/teaching_fragments.hpp"
int main() {
  using namespace slam_teaching;
  std::mt19937 rng(42);std::uniform_real_distribution<double> uniform(0,10);
  Points target,source;
  for (int i=0;i<100;++i) { const double x=uniform(rng),y=uniform(rng);target.emplace_back(x,y); }
  for (const auto & p:target) { source.push_back(rotation(0.3)*p+Vector2d(1,0.5)); }
  std::cout << std::fixed << std::setprecision(6);
  ICPWithAnalysis().analyze_convergence(source,target);corridor_matching_example();
  const Points fixed{{0,0},{1,0},{0,2},{1,3},{3,1},{4,5}};Points shifted;
  for (const auto & p:fixed) { shifted.push_back(rotation(0.03)*p+Vector2d(0.1,0.05)); }
  const auto robust=RobustScanMatcher().align(shifted,fixed);
  Vector3d expected;expected << -rotation(0.03).transpose()*Vector2d(0.1,0.05),-0.03;
  const MatrixXd map=MatrixXd::Constant(220,220,0.5),gradient=MatrixXd::Zero(220,220);
  const auto gn=GaussNewtonScanMatcher().optimize(fixed,Vector3d::Zero(),map,gradient,gradient);
  analyze_scan_matching(map,fixed);
  std::cout << "robust error=" << (robust-expected).norm() << " flat_gn=" << gn.transpose() << '\n';
  std::cout << generate_mapping_path("office") << '\n';
  std::vector<MCLParticle> heading(2);heading[1].pose.z()=0.05;apply_imu_heading(heading,0.0);
  LocalizationHealthMonitor health;std::vector<Vector2d> positions;
  for (int i=0;i<10;++i) { positions.emplace_back(i%2 ? -3.0 : 3.0,i%2 ? -3.0 : 3.0); }
  bool recovery=false;for (int i=0;i<11;++i) { recovery=health.needs_recovery(positions); }
  std::cout << "imu_weights=" << heading[0].weight << ',' << heading[1].weight << " recovery_after_11=" << recovery << '\n';
  const bool pass=recovery && std::abs(heading[1].weight-std::exp(-0.5))<1e-12 &&(robust-expected).norm()<1e-6 && gn.norm()==0 && RobustScanMatcher::huber_weight(2.0,0.5)==0.25;
  std::cout << "pass=" << std::boolalpha << pass << '\n';return pass ? 0 : 1;
}
