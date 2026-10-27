#pragma once
#include "slam_teaching_cpp/hector.hpp"
#include <random>
namespace slam_teaching {
// BEGIN RBPFSLAMParticle
struct RBPFSLAMParticle {
  Vector3d pose = Vector3d::Zero(); double weight = 1.0; MatrixXd map;
  std::vector<Vector3d> trajectory; double resolution;
  RBPFSLAMParticle(int rows = 200, int cols = 200, double res = 0.05)
  : map(MatrixXd::Zero(rows, cols)), resolution(res) {}
  void predict(const Vector2d & control, const Vector3d & noise, std::mt19937 & rng) {
    const double theta = pose.z();
    std::normal_distribution<double> normal(0.0, 1.0);
    const double nx = normal(rng), ny = normal(rng), nt = normal(rng);
    pose += Vector3d(control.x()*std::cos(theta)*0.1 + nx*noise.x(),
      control.x()*std::sin(theta)*0.1 + ny*noise.y(), control.y()*0.1 + nt*noise.z());
    pose.z() = wrap(pose.z()); trajectory.push_back(pose);
  }
  void update_map(const Points & scan) {
    for (const auto & point : scan) {
      Vector2d p = rotation(pose.z()) * point + pose.head<2>();
      int x = static_cast<int>(p.x()/resolution), y = static_cast<int>(p.y()/resolution);
      if (x >= 0 && y >= 0 && x < map.cols() && y < map.rows()) { map(y, x) += 1.0; }
    }
  }
  double compute_likelihood(const Points & scan, double sensor_noise = 0.1) const {
    (void)sensor_noise;  // 原示例没有使用此参数，仅乘各终点的占据概率。
    double likelihood = 1.0;
    for (const auto & point : scan) {
      Vector2d p = rotation(pose.z()) * point + pose.head<2>();
      int x = static_cast<int>(p.x()/resolution), y = static_cast<int>(p.y()/resolution);
      if (x >= 0 && y >= 0 && x < map.cols() && y < map.rows()) {
        likelihood *= 1.0/(1.0+std::exp(-map(y, x)));
      }
    }
    return likelihood;
  }
};
// END RBPFSLAMParticle

// BEGIN compute_effective_sample_size
inline double compute_effective_sample_size(const std::vector<double> & weights) {
  const double total = std::accumulate(weights.begin(), weights.end(), 0.0);
  if (total == 0.0) { return 0.0; }
  double squares = 0.0; for (double w : weights) { squares += (w/total)*(w/total); }
  return 1.0/squares;
}
inline bool detect_particle_depletion(const std::vector<RBPFSLAMParticle> & particles) {
  if (particles.empty()) { throw std::invalid_argument("粒子集合为空"); }
  std::vector<double> weights; for (const auto & p : particles) { weights.push_back(p.weight); }
  const double neff = compute_effective_sample_size(weights);
  std::cout << "有效粒子比: " << neff/particles.size() << " (Neff=" << neff << '/' << particles.size() << ")\n";
  return neff < particles.size()*0.5;
}
// END compute_effective_sample_size

// BEGIN FastSLAM1_0
class FastSLAM1_0 {
public:
  struct Landmark { Vector2d mean = Vector2d::Zero(); Eigen::Matrix2d cov = Eigen::Matrix2d::Zero(); bool seen = false; };
  struct Particle { Vector3d pose = Vector3d::Zero(); double weight; std::vector<Landmark> landmarks; };
  std::vector<Particle> particles; std::mt19937 rng{42};
  explicit FastSLAM1_0(int count = 100, int landmarks = 100) {
    for (int i = 0; i < count; ++i) { particles.push_back({Vector3d::Zero(), 1.0/count, std::vector<Landmark>(landmarks)}); }
  }
  void predict(const Vector2d & control, const Vector3d & noise) {
    std::normal_distribution<double> normal(0, 1);
    for (auto & p : particles) {
      double nx = normal(rng), ny = normal(rng), nt = normal(rng), theta = p.pose.z();
      p.pose += Vector3d(control.x()*std::cos(theta)*0.1+nx*noise.x(),
        control.x()*std::sin(theta)*0.1+ny*noise.y(), control.y()*0.1+nt*noise.z());
    }
  }
  double compute_likelihood(const Particle & p, const Vector2d & observation, int id, const Vector2d & noise) const {
    const auto & landmark = p.landmarks.at(id); if (!landmark.seen) { return 1.0; }
    Vector2d d = landmark.mean - p.pose.head<2>();
    Vector2d innovation(observation.x()-d.norm(), wrap(observation.y()-std::atan2(d.y(),d.x())+p.pose.z()));
    Eigen::Matrix2d S = landmark.cov + noise.asDiagonal().toDenseMatrix();
    if (S.determinant() <= 0) { return 0.001; }
    return std::max(0.001, std::exp(-0.5*innovation.dot(S.inverse()*innovation)) /
      (2.0*std::acos(-1.0)*std::sqrt(S.determinant())));
  }
  void update_weights(const Vector2d & observation, int id, const Vector2d & noise) {
    double total = 0.0;
    for (auto & p : particles) { p.weight *= compute_likelihood(p, observation, id, noise); total += p.weight; }
    for (auto & p : particles) { p.weight /= total; }
  }
  void resample() {
    const auto count = particles.size();
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    const double offset = uniform(rng); std::vector<Particle> output;
    std::size_t idx = 0; double cumulative = particles[0].weight;
    for (std::size_t i = 0; i < count; ++i) {
      const double position = (i+offset)/count;
      while (idx+1 < count && cumulative < position) { cumulative += particles[++idx].weight; }
      output.push_back(particles[idx]); output.back().weight = 1.0/count;
    }
    particles = std::move(output);
  }
};
// END FastSLAM1_0

// BEGIN FastSLAM2_0
class FastSLAM2_0 {
public:
  std::mt19937 rng{42};
  static double scan_match_score(const Vector3d & pose, const Points & scan, const MatrixXd & map) {
    if (scan.empty()) { return -std::numeric_limits<double>::infinity(); }
    double score = 0.0;
    for (const auto & point : scan) {
      Vector2d p = rotation(pose.z())*point + pose.head<2>();
      int x = static_cast<int>(p.x()/0.05), y = static_cast<int>(p.y()/0.05);
      if (x >= 0 && y >= 0 && x < map.cols() && y < map.rows()) {
        score += std::log(std::max(0.01, 1.0/(1.0+std::exp(-map(y,x)))));
      }
    }
    return score/scan.size();
  }
  Vector3d improved_proposal_distribution(const RBPFSLAMParticle & particle,
    const Vector2d & control, const Points & scan, const Vector3d & noise) {
    if (scan.empty()) { throw std::invalid_argument("提议分布需要非空扫描"); }
    std::vector<Vector3d> candidates; std::vector<double> weights;
    std::normal_distribution<double> normal(0, 1);
    for (int i = 0; i < 30; ++i) {
      const double nx = normal(rng), ny = normal(rng), nt = normal(rng), theta = particle.pose.z();
      Vector3d candidate = particle.pose + Vector3d(control.x()*std::cos(theta)*0.1+nx*noise.x(),
        control.x()*std::sin(theta)*0.1+ny*noise.y(), control.y()*0.1+nt*noise.z());
      candidates.push_back(candidate); weights.push_back(scan_match_score(candidate, scan, particle.map));
    }
    const double maximum = *std::max_element(weights.begin(), weights.end());
    for (auto & w : weights) { w = std::exp(w-maximum); }
    std::discrete_distribution<std::size_t> distribution(weights.begin(), weights.end());
    return candidates[distribution(rng)];
  }
};
// END FastSLAM2_0

// BEGIN GmappingProposalDistribution
class GmappingProposalDistribution {
public:
  std::mt19937 rng{42};
  std::pair<Vector3d, Eigen::Matrix3d> compute_optimized_proposal(const Vector3d & particle_pose,
    const Vector3d & odom, const Points & scan, const MatrixXd & map, double resolution) const {
    (void)odom;  // 沿用原片段：里程计参数尚未参与提议分布，不能视为完整 gmapping。
    Vector3d optimized;
    try { optimized = HectorScanMatcher().match(scan, map, resolution, particle_pose); }
    catch (const std::invalid_argument &) { return {particle_pose, Vector3d(0.5,0.5,0.3).asDiagonal()}; }
    Vector3d delta = optimized-particle_pose;
    Eigen::Matrix3d laser_cov = delta*delta.transpose() + 0.01*Eigen::Matrix3d::Identity();
    Eigen::Matrix3d odom_cov = Vector3d(0.02,0.02,0.01).asDiagonal();
    Eigen::Matrix3d cov = (odom_cov.inverse()+laser_cov.inverse()).inverse();
    return {cov*(odom_cov.inverse()*particle_pose+laser_cov.inverse()*optimized), cov};
  }
  Vector3d sample_from_proposal(const Vector3d & mean, const Eigen::Matrix3d & cov) {
    std::normal_distribution<double> normal(0, 1);
    const double x = normal(rng), y = normal(rng), z = normal(rng);
    return mean + cov.llt().matrixL()*Vector3d(x,y,z);
  }
};
// END GmappingProposalDistribution

// BEGIN AdaptiveResampler
class AdaptiveResampler {
public:
  double threshold_ratio = 0.5, neff = 0.0; int resample_count = 0; std::mt19937 rng{42};
  bool should_resample(const std::vector<RBPFSLAMParticle> & particles) {
    std::vector<double> weights; for (const auto & p : particles) { weights.push_back(p.weight); }
    neff = compute_effective_sample_size(weights);
    bool need = neff < particles.size()*threshold_ratio;
    if (need) { ++resample_count; } return need;
  }
  std::vector<RBPFSLAMParticle> stratified_resample(const std::vector<RBPFSLAMParticle> & particles) {
    if (particles.empty()) { throw std::invalid_argument("粒子集合为空"); }
    double total = 0; for (const auto & p : particles) { total += p.weight; }
    if (total <= 0) { throw std::invalid_argument("重采样权重总和必须为正"); }
    std::uniform_real_distribution<double> uniform(0.0,1.0); std::vector<RBPFSLAMParticle> output;
    for (std::size_t i = 0; i < particles.size(); ++i) {
      double position = (i+uniform(rng))/particles.size(), sum = 0;
      std::size_t j = 0;
      for (; j+1 < particles.size(); ++j) { sum += particles[j].weight/total; if (sum >= position) { break; } }
      output.push_back(particles[j]); output.back().weight = 1.0/particles.size();
    }
    return output;
  }
};
// END AdaptiveResampler

// BEGIN GmappingSLAM
class GmappingSLAM {
public:
  std::vector<RBPFSLAMParticle> particles; AdaptiveResampler resampler; GmappingProposalDistribution proposal;
  MatrixXd best_map; double resolution; bool map_updated = false;
  explicit GmappingSLAM(int count = 30, int rows = 200, int cols = 200, double res = 0.05)
  : best_map(MatrixXd::Zero(rows, cols)), resolution(res) {
    for (int i = 0; i < count; ++i) { particles.emplace_back(rows, cols, res); particles.back().weight = 1.0/count; }
  }
  void process_scan(const Points & scan, const Vector3d & odometry) {
    double total = 0;
    for (auto & p : particles) {
      MatrixXd probability = (1.0/(1.0+(-p.map.array()).exp())).matrix();
      auto distribution = proposal.compute_optimized_proposal(p.pose, odometry, scan, probability, resolution);
      p.pose = proposal.sample_from_proposal(distribution.first, distribution.second); p.trajectory.push_back(p.pose);
      double score = 0; int count = 0;
      for (const auto & point : scan) {
        Vector2d world = rotation(p.pose.z())*point+p.pose.head<2>();
        int x = static_cast<int>(world.x()/resolution), y = static_cast<int>(world.y()/resolution);
        if (x >= 0 && y >= 0 && x < p.map.cols() && y < p.map.rows()) {
          score += std::log(std::max(0.01, probability(y,x))); ++count;
          p.map(y,x) = std::min(5.0, p.map(y,x)+0.5);
        }
      }
      p.weight *= std::exp(score/std::max(count,1))+1e-10; total += p.weight;
    }
    if (total > 0) { for (auto & p : particles) { p.weight /= total; } }
    // 重采样会重置权重，先保存观测权重最高粒子的地图。
    best_map = std::max_element(particles.begin(), particles.end(),
      [](const auto & a, const auto & b) { return a.weight < b.weight; })->map;
    if (resampler.should_resample(particles)) { particles = resampler.stratified_resample(particles); }
    map_updated = true;
  }
  std::vector<int8_t> get_occupancy_grid() const {
    std::vector<int8_t> output;
    for (int y = 0; y < best_map.rows(); ++y) { for (int x = 0; x < best_map.cols(); ++x) {
      double p = 1.0/(1.0+std::exp(-best_map(y,x))); output.push_back(p > 0.7 ? 100 : p < 0.3 ? 0 : -1);
    }} return output;
  }
};
// END GmappingSLAM

// BEGIN suggest_particle_count
inline int suggest_particle_count(double environment_size, double sensor_noise) {
  const double size_factor = environment_size < 100 ? 0.5 : environment_size < 500 ? 1.0 : environment_size < 2000 ? 2.0 : 3.0;
  return std::clamp(static_cast<int>(30*size_factor*(1.0+sensor_noise*5)), 20, 200);
}
// END suggest_particle_count
}  // namespace slam_teaching
