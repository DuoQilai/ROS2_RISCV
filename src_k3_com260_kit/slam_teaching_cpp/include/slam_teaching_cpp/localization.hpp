#pragma once
#include "slam_teaching_cpp/particles.hpp"
#include <set>
#include <tuple>
namespace slam_teaching {
struct MCLParticle { Vector3d pose = Vector3d::Zero(); double weight = 1.0; };
inline MatrixXd distance_field(const MatrixXd & map, double resolution, double no_obstacle) {
  Points obstacles;
  for (int y = 0; y < map.rows(); ++y) { for (int x = 0; x < map.cols(); ++x) {
    if (map(y,x) == 100) { obstacles.emplace_back(x,y); }
  }}
  MatrixXd output = MatrixXd::Constant(map.rows(), map.cols(), no_obstacle);
  if (obstacles.empty()) { return output; }
  for (int y = 0; y < map.rows(); ++y) { for (int x = 0; x < map.cols(); ++x) {
    Vector2d p(x,y); output(y,x) = (p-obstacles[nearest(p, obstacles)]).norm()*resolution;
  }} return output;
}
// BEGIN MonteCarloLocalization
class MonteCarloLocalization {
public:
  int num_particles; std::vector<MCLParticle> particles; MatrixXd map_data, distances;
  double resolution = 0; Vector2d origin = Vector2d::Zero(); std::mt19937 rng{42};
  explicit MonteCarloLocalization(int count = 500) : num_particles(count) {}
  void initialize_global(const MatrixXd & map, double res, const Vector2d & offset) {
    map_data = map; resolution = res; origin = offset; distances = distance_field(map, res, 100.0);
    Points free;
    for (int y = 0; y < map.rows(); ++y) { for (int x = 0; x < map.cols(); ++x) {
      if (map(y,x) == 0) { free.emplace_back(x,y); }
    }}
    if (free.empty()) { throw std::invalid_argument("地图中没有自由栅格"); }
    std::uniform_int_distribution<std::size_t> index(0, free.size()-1);
    std::uniform_real_distribution<double> xy(-0.1,0.1), angle(-std::acos(-1.0),std::acos(-1.0));
    particles.clear();
    for (int i = 0; i < num_particles; ++i) {
      Vector2d point = free[index(rng)]*resolution+origin;
      double dx = xy(rng), dy = xy(rng), yaw = angle(rng);
      particles.push_back({Vector3d(point.x()+dx,point.y()+dy,yaw),1.0/num_particles});
    }
  }
  void initialize_pose(double x, double y, double yaw, const Vector3d & noise = Vector3d(0.1,0.1,0.1)) {
    std::normal_distribution<double> normal(0,1); particles.clear();
    for (int i = 0; i < num_particles; ++i) {
      const double nx=normal(rng), ny=normal(rng), nt=normal(rng);
      particles.push_back({Vector3d(x+nx*noise.x(),y+ny*noise.y(),yaw+nt*noise.z()),1.0/num_particles});
    }
  }
  void predict(const Vector2d & control, const Vector3d & noise) {
    std::normal_distribution<double> normal(0,1);
    for (auto & p : particles) {
      const double nx=normal(rng), ny=normal(rng), nt=normal(rng), theta=p.pose.z();
      p.pose += Vector3d(control.x()*std::cos(theta)*0.1+nx*noise.x(),
        control.x()*std::sin(theta)*0.1+ny*noise.y(),control.y()*0.1+nt*noise.z());
    }
  }
  double compute_scan_likelihood(const Vector3d & pose, const Points & scan) const {
    if (map_data.size() == 0) { return 1.0; }
    double score=0; int count=0;
    for (const auto & point : scan) {
      Vector2d grid=(rotation(pose.z())*point+pose.head<2>()-origin)/resolution;
      int x=static_cast<int>(grid.x()), y=static_cast<int>(grid.y());
      if (x>=0 && y>=0 && x<distances.cols() && y<distances.rows()) {
        double d=distances(y,x), likelihood=std::exp(-d*d/(2*0.1*0.1))/(std::sqrt(2*std::acos(-1.0))*0.1);
        score+=std::log(std::max(likelihood,1e-10)); ++count;
      }
    }
    return count ? std::exp(score/count) : 1e-10;
  }
  void update_weights(const Points & scan) {
    double total=0;
    for (auto & p : particles) { p.weight*=compute_scan_likelihood(p.pose,scan)+1e-10; total+=p.weight; }
    if (total>0) { for (auto & p : particles) { p.weight/=total; } }
  }
};
// END MonteCarloLocalization

// BEGIN beam_model_likelihood
inline double beam_model_likelihood(double z, double expected, double max_range,
  double zhit=0.95, double zshort=0.01, double zmax=0.01, double zrand=0.03) {
  const double sigma=0.1;
  const double hit=zhit*std::exp(-(z-expected)*(z-expected)/(2*sigma*sigma))/(std::sqrt(2*std::acos(-1.0))*sigma);
  const double short_return=z<expected ? zshort*2.0/expected*(1.0-z/expected) : 0.0;
  return hit+short_return+(z>=max_range-0.01 ? zmax : 0.0)+zrand/max_range;
}
// END beam_model_likelihood

// BEGIN LikelihoodFieldModel
class LikelihoodFieldModel {
public:
  MatrixXd distances; double resolution, sigma=0.1, z_hit=0.95, z_rand=0.05, z_max=30.0;
  LikelihoodFieldModel(const MatrixXd & map, double res) : distances(distance_field(map,res,30.0)), resolution(res) {}
  double probability(const Vector2d & point, const Vector3d & pose) const {
    Vector2d grid=(rotation(pose.z())*point+pose.head<2>())/resolution;
    int x=static_cast<int>(grid.x()), y=static_cast<int>(grid.y());
    if (x<0 || y<0 || x>=distances.cols() || y>=distances.rows()) { return z_rand/z_max; }
    double d=distances(y,x); if (d==0) { return 1.0; }
    return z_hit*std::exp(-d*d/(2*sigma*sigma))/(std::sqrt(2*std::acos(-1.0))*sigma)+z_rand/z_max;
  }
};
// END LikelihoodFieldModel

// BEGIN low_variance_resampling
inline std::vector<MCLParticle> low_variance_resampling(const std::vector<MCLParticle> & particles,
  std::mt19937 & rng, int num=0) {
  if (particles.empty()) { throw std::invalid_argument("粒子集合为空"); }
  if (num==0) { num=static_cast<int>(particles.size()); }
  double total=0; for (const auto & p : particles) { total+=p.weight; }
  if (total<=0 || num<=0) { throw std::invalid_argument("重采样需要正粒子数和正权重和"); }
  std::uniform_real_distribution<double> uniform(0.0,1.0/num);
  double position=uniform(rng), cumulative=particles[0].weight/total; std::size_t idx=0;
  std::vector<MCLParticle> output;
  for (int i=0;i<num;++i,position+=1.0/num) {
    while (idx+1<particles.size() && cumulative<position) { cumulative+=particles[++idx].weight/total; }
    output.push_back({particles[idx].pose,1.0/num});
  }
  return output;
}
// END low_variance_resampling

// BEGIN KLDSampling
class KLDSampling {
public:
  int min_particles=100, max_particles=2000; double epsilon=0.05, delta=0.01; std::mt19937 rng{42};
  int compute_kld_sample_size(const std::vector<MCLParticle> & particles) const {
    if (particles.size()<=static_cast<std::size_t>(min_particles)) { return min_particles; }
    std::set<std::tuple<int,int,int>> bins;
    for (const auto & p : particles) { bins.emplace(static_cast<int>(p.pose.x()/0.2),
      static_cast<int>(p.pose.y()/0.2),static_cast<int>(p.pose.z()/0.1)); }
    // 原文的简化近似式，固定 z_score=2.57；不是 Nav2 的完整 KLD 上界公式。
    return std::clamp(static_cast<int>((bins.size()-1)/(2*epsilon)*2.57*2.57),min_particles,max_particles);
  }
  std::vector<MCLParticle> adaptive_resample(const std::vector<MCLParticle> & particles) {
    std::vector<double> weights; for (const auto & p : particles) { weights.push_back(p.weight); }
    if (compute_effective_sample_size(weights)>=particles.size()*0.5) { return particles; }
    auto output=low_variance_resampling(particles,rng,compute_kld_sample_size(particles));
    std::uniform_real_distribution<double> uniform(0,1);
    if (uniform(rng)<0.05) {
      std::uniform_int_distribution<std::size_t> index(0,output.size()-1);
      std::uniform_real_distribution<double> xy(-1.0,1.0), angle(-0.5,0.5);
      for (std::size_t i=0;i<std::max<std::size_t>(1,output.size()/20);++i) {
        auto & p=output[index(rng)]; p.pose.x()+=xy(rng); p.pose.y()+=xy(rng); p.pose.z()+=angle(rng);
      }
    }
    return output;
  }
};
// END KLDSampling
}  // namespace slam_teaching
