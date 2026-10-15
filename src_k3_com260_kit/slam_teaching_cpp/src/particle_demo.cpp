#include <iomanip>
#include <iostream>
#include "slam_teaching_cpp/localization.hpp"
int main() {
  using namespace slam_teaching;
  std::cout << std::fixed << std::setprecision(6);
  std::mt19937 rng(42);
  RBPFSLAMParticle rbpf(20,20,0.05); rbpf.predict(Vector2d(1,0.2),Vector3d::Zero(),rng);
  rbpf.update_map({Vector2d(0.2,0)});
  std::cout << "rbpf pose=" << rbpf.pose.transpose() << " trajectory=" << rbpf.trajectory.size() << '\n';
  const double neff_uniform=compute_effective_sample_size({1,1,1,1});
  const double neff_single=compute_effective_sample_size({1,0,0,0});
  std::cout << "Neff uniform=" << neff_uniform << " single=" << neff_single << '\n';
  FastSLAM1_0 fast(4,1); fast.predict(Vector2d(1,0),Vector3d::Zero());
  fast.update_weights(Vector2d(1,0),0,Vector2d(0.1,0.1)); fast.resample();
  GmappingProposalDistribution proposal;
  auto distribution=proposal.compute_optimized_proposal(Vector3d::Zero(),Vector3d::Zero(),
    {Vector2d(0.1,0.1)},MatrixXd::Constant(20,20,0.5),0.05);
  MatrixXd map=MatrixXd::Zero(5,5); map(2,2)=100;
  LikelihoodFieldModel field(map,0.1);
  MonteCarloLocalization mcl(4); mcl.initialize_global(map,0.1,Vector2d::Zero());
  mcl.initialize_pose(0,0,0,Vector3d::Zero()); mcl.predict(Vector2d(1,0),Vector3d::Zero());
  mcl.update_weights({Vector2d(0.1,0.2)});
  auto samples=low_variance_resampling(mcl.particles,rng,8);
  std::vector<MCLParticle> bins(101); for (std::size_t i=50;i<bins.size();++i) { bins[i].pose.x()=0.3; }
  const int kld=KLDSampling().compute_kld_sample_size(bins);
  double beam=beam_model_likelihood(1.0,1.0,30.0);
  std::cout << "proposal mean=" << distribution.first.transpose() << " cov=" << distribution.second.diagonal().transpose() << '\n';
  std::cout << "distance(0,0)=" << field.distances(0,0) << " hit=" << field.probability(Vector2d(0.2,0.2),Vector3d::Zero())
    << " beam=" << beam << " resampled=" << samples.size() << " kld=" << kld << '\n';
  const bool pass=std::abs(rbpf.pose.x()-0.1)<1e-12 && std::abs(rbpf.pose.z()-0.02)<1e-12 &&
    neff_uniform==4 && neff_single==1 && std::abs(distribution.second(0,0)-1.0/150)<1e-12 &&
    std::abs(field.distances(0,0)-std::sqrt(0.08))<1e-12 && samples.size()==8 && kld==100 &&
    std::abs(beam-(0.95/(std::sqrt(2*std::acos(-1.0))*0.1)+0.001))<1e-12;
  std::cout << "pass=" << std::boolalpha << pass << '\n'; return pass ? 0 : 1;
}
