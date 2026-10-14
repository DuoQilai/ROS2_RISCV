# 第13章 gmapping粒子滤波SLAM

本章为 COM260 Kit 的 C++17 教学副本。课程概念、练习及数值要求沿用原章；实测状态与素材见[实验第10章](../lab_manuals_k3_com260_kit/ch10_lab.md)和[运行证据](../lab_manuals_k3_com260_kit/runtime_evidence.md#ch10)。算法片段展示相应数学步骤，不能代替完整 SLAM 系统。

数值示例和 ROS 已知输入已分别在 COM260 与 x86 构建运行。原始记录：[COM260数值](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-numerics.cast)、[COM260 ROS接口](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-ros.cast)、[真实仿真传感器](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-live-sensors.cast)。这些检查的输入和范围分开记录，不能据此宣称所有教学算法均在真实场景达到定位精度要求。

教学程序位于 [`slam_teaching_cpp`](../src_k3_com260_kit/slam_teaching_cpp/README.md)。COM260 终端先执行：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
```


> **课程**：ROS2 C++17 编程
> **章节**：第13章
> **课时**：2 课时（90 分钟）
> **教学方式**：讲授 + 演示

---

## 学习目标

本章学习目标包括：理解粒子滤波SLAM的基本原理和Rao-Blackwellized分解，掌握FastSLAM框架的算法流程，熟悉gmapping的提议分布优化和自适应重采样策略，能够在ROS2中使用slam_toolbox进行建图，掌握建图参数调优方法。

## 13.1 粒子滤波SLAM原理

### 13.1.1 粒子滤波基本概念

粒子滤波 (Particle Filter) 是一种基于蒙特卡洛方法的贝叶斯滤波实现。它使用一组加权粒子来近似后验概率分布。

**粒子的定义：**每个粒子表示一个假设的机器人轨迹；粒子包含位姿 `x_t` 和权重 `w_t`；所有粒子共同近似后验分布 `p(x_t | z_{1:t}, u_{1:t-1})`。

**粒子滤波的优势：**可以表示任意复杂的概率分布（非高斯、多模态），易于实现、不需要线性化，计算复杂度可控（通过粒子数平衡精度和效率）。

**粒子滤波的三大步骤：**
```
1. 预测 (Prediction):   x_t ~ p(x_t | x_{t-1}, u_t)
2. 更新 (Update):       w_t = w_{t-1} · p(z_t | x_t)
3. 重采样 (Resampling): 根据权重重新采样粒子
```

### 13.1.2 粒子滤波的SLAM应用

在SLAM中，状态空间包含机器人的完整路径和环境地图，维度极高。直接使用粒子滤波估计完整SLAM后验是不现实的。

**解决方案：Rao-Blackwellized Particle Filter (RBPF)**

RBPF利用条件独立性将SLAM分解为两个子问题：

```
p(x_{1:t}, m | z_{1:t}, u_{1:t-1}) =
p(m | x_{1:t}, z_{1:t}) · p(x_{1:t} | z_{1:t}, u_{1:t-1})
```

分解含义：使用粒子滤波估计机器人路径 `x_{1:t}`，每个粒子根据其估计的路径独立维护一个地图 `m`，给定路径后地图估计可以解析计算（条件高斯）。

以下类/函数片段摘自 [`particles.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/particles.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

原例注释提到Bresenham，但实际只更新终点；此处保留终点更新行为，地图原点仍为(0,0)。sensor_noise在原似然计算中未使用。

```cpp
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
```

### 13.1.3 粒子退化问题

粒子滤波面临的核心挑战是**粒子退化**：经过若干次迭代后，大部分粒子的权重会变得极小，只有少数粒子具有显著的权重。

**退化现象的度量：**

以下类/函数片段摘自 [`particles.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/particles.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
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
```

**解决退化的方法：重采样**

### 13.1.4 官方要点——RBPF 的理论根基与 Grisetti 2007 年关键论文

本章 13.1.2 的 Rao-Blackwellized 分解对应的是 Grisetti、Stachniss 与 Burgard 于 2007 年发表的经典论文《Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters》（TER 期刊）及其 2005 年 ICRA 会议版本。论文提出的三项改进正是本章 13.3 的三板斧：①用激光似然优化提议分布（13.3.2，只在观测信息量大时用激光修正采样，否则退回里程计模型）；②自适应重采样（13.3.3，用有效样本数 Neff 阈值替代每帧重采样，避免粒子多样性骤减）；③选择性扫描匹配（仅当移动/旋转超过阈值才执行）。理解本章代码后重读该论文的推导，是深入粒子滤波 SLAM 最直接的一条路径。

## 13.2 FastSLAM框架

### 13.2.1 FastSLAM 1.0

FastSLAM 1.0 是最早将RBPF应用于SLAM的算法：

```
FastSLAM 1.0 算法流程:

每接收到一组观测和控制:
1. 对每个粒子:
   a. 根据运动模型采样新位姿
   b. 计算观测似然（权重）
2. 归一化粒子权重
3. 根据权重重采样
4. 对每个粒子更新EKF路标点观测
```

以下类/函数片段摘自 [`particles.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/particles.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

本片段覆盖运动预测、已知路标似然和低方差重采样；原例没有路标初始化/路标EKF更新步骤，这些仍是课程待实现部分，不能称为完整FastSLAM。未见路标的似然仍为1。

```cpp
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
```

### 13.2.2 FastSLAM 2.0

FastSLAM 2.0 的关键改进是：使用激光观测优化提议分布，而不仅仅依赖运动模型。

**改进点：**提议分布融合当前观测信息，直接在提议分布中融入激光观测，使采样更集中到高概率区域：

  ```
  x_t ~ p(x_t | x_{t-1}, u_t, z_t, m)
  ```

以下类/函数片段摘自 [`particles.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/particles.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

保留30个候选、0.1 s运动模型、0.05 m栅格、指数归一化提议分布。这里只展示优化采样步骤，不包含完整FastSLAM2.0循环。

```cpp
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
```

### 13.2.3 FastSLAM vs EKF-SLAM

| 特性 | FastSLAM (粒子滤波) | EKF-SLAM |
|------|-------------------|-----------|
| 状态表示 | 离散粒子群 | 高斯分布 |
| 非高斯分布 | 支持 | 不支持 |
| 数据关联 | 每个粒子独立处理 | 需要全局关联 |
| 计算复杂度 | O(M·N) M粒子数,N路标数 | O(N²) |
| 实现难度 | 中等 | 简单 |
| 应用场景 | 大场景、非高斯  | 小场景、高斯噪声 |

### 13.2.4 官方要点——粒子滤波的工程代价与替代路线

粒子滤波的三重代价在 galactic 之后的文献与 The Construct 课程中被反复讨论：粒子多样性丧失（重采样后粒子快速趋同，对应 13.1.3 的退化度量 Neff）、计算量随粒子数线性增长、以及对里程计漂移的敏感。RBPF 在里程计差时会让提议分布严重偏离，粒子被迫用数量弥补——这正是本章 13.5.1 建议「噪声大→加粒子」的原因。

作为替代，图优化 SLAM（slam_toolbox 的 pose graph + Ceres、Cartographer 的前端配准+后端优化）把「一次解算近似」换成「最小二乘再求解」，不依赖粒子数量，劣势是把漂移责任转移给回环检测与参数调优。Robotics Back-End 的对比文章给出结论性建议：中小场景、REAL 低噪声里程计上 RBPF 已够用；场地大、要求回环修正时直接上 slam_toolbox（本章 13.4 的实践路线）。

## 13.3 gmapping实现详解

### 13.3.1 gmapping算法特点

gmapping 是基于RBPF的2D SLAM算法，在FastSLAM基础上做了重要改进：

**核心改进：**使用激光观测的似然来优化粒子的提议分布，使粒子集中在高概率区域；仅在必要时进行自适应重采样，减少粒子退化；只对关键帧进行选择性扫描匹配，提高效率。

**gmapping的算法流程：**
```
1. 读取激光扫描数据
2. 根据运动模型预测粒子位姿
3. 计算每个粒子的权重（观测似然）
4. 使用优化提议分布重新采样
5. 更新每个粒子的占据栅格地图
6. 自适应重采样决策
7. 发布建图结果
```

### 13.3.2 gmapping的提议分布优化

**标准粒子滤波的问题：** 提议分布仅基于运动模型，当运动模型噪声大而观测噪声小时，采样效率低。

**gmapping的改进：** 使用激光观测来"修正"提议分布，在观测似然尖锐时使用激光数据。

以下类/函数片段摘自 [`particles.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/particles.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

原例接收odom但没有使用它；本片段如实保留该边界，不能据此声称完成里程计融合。匹配输入应是占据概率，而非未转换的log-odds。

```cpp
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
```

### 13.3.3 自适应重采样

gmapping使用**Neff（有效样本数）**来判断是否需要进行重采样：

以下类/函数片段摘自 [`particles.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/particles.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

原例混用了对象属性p.weight和字典p[weight]，C++统一为结构体，避免重采样后类型不匹配。分层重采样复制独立地图，权重重置为1/N。全零权重无法定义采样分布，报错而非生成空集合。

```cpp
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
```

### 13.3.4 完整gmapping实现

以下类/函数片段摘自 [`particles.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/particles.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

保留原章节标题，但以下仅是原文简化流程的可运行教学核心：没有自由射线更新，也没有完整里程计融合和工程级扫描匹配。它不等同于ROS1 gmapping或ROS2 slam_toolbox。修正两处会破坏演示的问题：传给匹配器前将log-odds转为概率；在重采样重置权重前保存最佳地图。真实建图采用本章后续已提供的slam_toolbox入口。

```cpp
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
```

## 13.4 ROS2中的建图工具

### 13.4.1 slam_toolbox概述

在ROS2中，slam_toolbox替代了ROS1的gmapping，提供了更丰富的功能：

**slam_toolbox支持的工作模式：**`mapping` 为在线建图模式，`localization` 在已有地图上定位，`mapping+localization` 同时建图和定位，`localize_in_partial_map` 支持部分地图定位。

**slam_toolbox的核心参数：**

```yaml
slam_toolbox:
  ros__parameters:
    mode: mapping
    solver_plugin: solver_plugins::CeresSolver
    map_frame: map
    odom_frame: odom
    base_frame: base_footprint
    scan_topic: /scan
    resolution: 0.05
    map_update_interval: 1.0
    minimum_time_interval: 0.2
    transform_timeout: 0.2
    max_laser_range: 8.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_big: 0.003
    correlation_search_space_smear_deviation: 0.03
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
```

### 13.4.2 slam_toolbox参数详解

**建图相关参数：**

| 参数 | 默认值 | 说明 | 调优建议 |
|------|--------|------|---------|
| resolution | 0.05 | 地图分辨率(m) | 大场景0.1，精细0.025 |
| map_update_interval | 1.0 | 地图更新间隔(秒) | 移动快则减小 |
| minimum_time_interval | 0.2 | 最小帧间隔(秒) | 根据激光频率调整 |
| max_laser_range | 8.0 | 最大有效激光距离(m) | 根据激光型号设置 |

**回环检测参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| do_loop_closing | true | 启用回环检测 |
| loop_match_minimum_chain_size | 10 | 最小闭环匹配链大小 |
| loop_match_maximum_variance_big | 0.003 | 闭环匹配方差阈值 |
| correlation_search_space_smear_deviation | 0.03 | 搜索空间涂片偏差 |

**扫描匹配参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| minimum_travel_distance | 0.3 | 触发匹配的最小移动距离(m) |
| minimum_travel_heading | 0.3 | 触发匹配的最小旋转角度(rad) |
| scan_matcher_ceres_initial_scale | 1.0 | Ceres匹配初始尺度 |
| scan_matcher_ceres_scale_step | 1.25 | Ceres匹配尺度步长 |

### 13.4.3 建图操作流程

先按本章末尾仿真实例准备 x86 课程容器。每条常驻命令使用独立终端：

**1. x86课程容器：启动仿真环境**

```bash
source /workspace/install/setup.bash
ros2 launch sim_demo sim_bringup.launch.py \
  world:="$(ros2 pkg prefix sim_demo)/share/sim_demo/worlds/museum.sdf" \
  sensor_profile:=teacher gui:=true rviz:=true
```

**2. x86课程容器：启动建图**

```bash
source /workspace/install/setup.bash
ros2 launch slam_lab online_mapping.launch.py use_sim_time:=true
```

**3. COM260：控制机器人探索环境**

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**4. x86课程容器：保存地图**

```bash
source /workspace/install/setup.bash
mkdir -p /workspace/output/maps
ros2 run nav2_map_server map_saver_cli -f /workspace/output/maps/my_map --ros-args -p use_sim_time:=true
```

检查生成的 `.pgm` 与 `.yaml`，对照当次 RViz 地图。先停止运动，再停止建图与仿真。

### 13.4.4 建图质量控制

完整程序：[`map_quality_monitor.cpp`](../src_k3_com260_kit/slam_teaching_cpp/src/map_quality_monitor.cpp)。



```cpp
#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
class MapQualityMonitor : public rclcpp::Node {
public:
  MapQualityMonitor() : Node("map_quality_monitor") {
    sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",10,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        if (++count_%10!=0 || msg->data.empty()) { return; }
        const auto free=std::count(msg->data.begin(),msg->data.end(),0);
        const auto occupied=std::count(msg->data.begin(),msg->data.end(),100);
        const auto unknown=std::count_if(msg->data.begin(),msg->data.end(),[](int v){return v<0;});
        const double coverage=100.0*(free+occupied)/msg->data.size();
        RCLCPP_INFO(get_logger(),"建图质量 [#%zu]: 覆盖=%.1f%% (空闲=%td, 占据=%td, 未知=%td) 分辨率=%.3fm",
          count_,coverage,free,occupied,unknown,msg->info.resolution);
        if (coverage>95) { RCLCPP_INFO(get_logger(),"环境已充分探索，建议保存地图!"); }
      });
  }
private:
  std::size_t count_=0; rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<MapQualityMonitor>()); rclcpp::shutdown();
}
```

上例 `coverage>95` 的提示只表示当前地图矩形内超过 95% 的栅格被该统计规则归类；它不包含地图外空间，也不能单凭该值证明整个环境已探索完成。

### 13.4.5 从gmapping到slam_toolbox

ROS2 课程使用 slam_toolbox 演示建图运行流程，但它基于扫描匹配和位姿图，与 gmapping 的粒子滤波结构不同。以下仅比较调参目的，不能把参数名直接互换；gmapping 的教学算法与原数值保留。

| gmapping参数 | slam_toolbox中可比较的设置 | 说明 |
|-------------|-----------------|------|
| particles | 无直接等价项 | slam_toolbox 不维护 gmapping 粒子集；`solver_plugin` 选择图优化求解器，不控制粒子数 |
| iterations | 无直接等价项 | `minimum_time_interval` 控制同步模式处理扫描的最小时间间隔，不是迭代次数 |
| srr/srt/stt | 无直接等价项 | 粒子运动采样噪声与图优化中的里程计约束不是同一参数模型 |
| llsamplerange/llsamplestep | correlation_search_space_* | 可比较搜索范围/分辨率的作用，但搜索算法和数值语义不同 |
| lsigma | 无直接等价项 | 激光似然宽度不能直接换成 Ceres 损失函数类型 |
| ogain | 无直接等价项 | 粒子权重平滑与图优化残差权重不是同一操作 |
| linearUpdate | minimum_travel_distance | 都可用于比较平移触发阈值，具体更新规则仍取决于后端 |
| angularUpdate | minimum_travel_heading | 都可用于比较旋转触发阈值，不能推定结果相同 |

参数定义见 [slam_toolbox Humble 配置说明](https://github.com/SteveMacenski/slam_toolbox/blob/humble/README.md#configuration)。

```bash
# ROS1 gmapping启动命令 (对比参考)
rosrun gmapping slam_gmapping scan:=/scan _particles:=30 _linearUpdate:=0.1

# x86课程容器：ROS2 slam_toolbox启动命令
source /workspace/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$(ros2 pkg prefix slam_lab)/share/slam_lab/config/mapper_params_online_async.yaml" \
  use_sim_time:=true
```

### 13.4.6 官方要点——gmapping 的官方文档与参数体系

gmapping 是 ROS 1 时代应用最广的 2D SLAM 包，其 ROS Wiki 页面提供了完整的参数表，与本章 13.4.5 的映射关系一一对应：`particles`（粒子数）、`linearUpdate`/`angularUpdate`（触发扫描匹配的移动/旋转阈值，对应 slam_toolbox 的 `minimum_travel_distance`/`minimum_travel_heading`）、`srr`/`srt`/`str`/`stt`（里程计运动模型噪声）、`lsigma`/`ogain`（激光似然参数）、`llsamplerange`（局部搜索范围）。

openslam.org 的 gmapping 项目页则给出原始实现的说明：它要求输入 `/scan`、里程计与 TF（`map`→`odom`→`base`→`laser`），并强调「粒子数越多建图越稳定，但每帧代价线性增长」——这是本章 13.5.1 粒子数选择公式的理论根据。官方还指出 gmapping 对大场景有地图尺寸限制（内存预分配），这也是 ROS 2 生态最终转向 slam_toolbox 的原因之一。

## 13.5 参数调优与最佳实践

### 13.5.1 粒子数选择

粒子数量直接影响建图质量和计算效率：

以下类/函数片段摘自 [`particles.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/particles.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
inline int suggest_particle_count(double environment_size, double sensor_noise) {
  const double size_factor = environment_size < 100 ? 0.5 : environment_size < 500 ? 1.0 : environment_size < 2000 ? 2.0 : 3.0;
  return std::clamp(static_cast<int>(30*size_factor*(1.0+sensor_noise*5)), 20, 200);
}
```

### 13.5.2 建图路径规划

好的建图路径可以显著提高建图质量：

以下类/函数片段摘自 [`analysis_examples.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/analysis_examples.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
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
```

### 13.5.3 常见问题排查

**问题1：地图重影/偏移**

原因：回环检测未正确触发，或扫描匹配错误。

解决：降低移动速度并手动控制速度（线性 0.15、角速度 0.3），同时减少环境速度相关参数：

```bash
# COM260：手动控制速度，线性0.15 m/s、角速度0.3 rad/s
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.3
```

在 `slam_toolbox.ros__parameters` 下调整下列 YAML 项，重启建图后比较结果：

```yaml
minimum_travel_distance: 0.15
minimum_travel_heading: 0.15
```

**问题2：地图缺失区域**

原因：未充分探索，激光扫描范围有限。

解决：手动补充扫描并检查激光范围设置。自动探索是扩展练习，本课程未提供原示例中的 `exploration/exploration_node`，不能直接运行该入口。

```yaml
max_laser_range: 12.0
```

**问题3：定位丢失**

原因：快速移动或环境特征不足。

解决：增加角点检测灵敏度：

  ```yaml
  # 增加角点检测灵敏度
  correlation_search_space_smear_deviation: 0.05
  loop_match_maximum_variance_big: 0.005
  ```

### 13.5.4 官方要点——粒子滤波 SLAM 的调优与质量监控纪律

结合官方文档与课程实践，粒子滤波建图的三条纪律值得牢记：第一，录制 bag（本章开头实例的 `/scan`、`/odom`、`/tf`）离线试参，先检查 `ros2 bag info` 确认话题频率与时间戳连续，再谈算法；第二，用本章 13.4.4 的 MapQualityMonitor 思路量化覆盖度（unknown 比例）与重影（同一区域占据栅格离散度），而不是凭肉眼判断；第三，SimTime 环境强调 `use_sim_time:=true` 与时钟对齐，仿真中更容易复现的时间戳错位问题在真机上会因网络延迟放大。建议按本章练习第 4 题系统对比 `resolution`、`minimum_travel_distance` 与回环参数对最终地图的影响，并记录每组参数下的覆盖度数值。

## 课后练习

1. **原理题:** 说明Rao-Blackwellized粒子滤波如何将SLAM问题分解，为什么这种分解可以降低状态空间的维度？

2. **编程题:** 实现一个简化版的gmapping粒子滤波SLAM系统，包含粒子初始化、运动模型预测、权重计算和重采样四个核心步骤。

3. **分析题:** 比较gmapping和slam_toolbox在算法原理上的异同，说明为什么ROS2选择了slam_toolbox而不是继续使用gmapping。

4. **配置题:** 在Gazebo仿真中使用slam_toolbox进行建图，配置参数使得建图质量最优，包括分辨率、更新频率、回环检测等。

5. **推导题:** 推导gmapping中优化提议分布的数学形式，说明为什么融合激光观测的提议分布可以提高采样效率。

6. **设计题:** 某2000m²的仓库需要高精度建图，机器人配备2D激光雷达和轮式里程计。设计完整的建图方案，包括SLAM算法选择、参数配置、建图路径规划和地图质量评估方法。

---

## 仿真结合实例（当前仓库）：用 Gazebo 数据验证粒子滤波 SLAM 接口

### 目标与知识点对应

gmapping 的粒子滤波更新需要激光观测、里程计和 TF。本仓库没有 gmapping 节点，实例用 `sim_demo` 产生同样的数据接口，并用 rosbag 检查数据连续性。

### 运行步骤

仿真启动在 **x86 课程容器终端**执行，其他观测命令在 **COM260** 执行。x86 宿主仓库根目录先运行（每轮使用不同 RUN_ID）：

```bash
RUN_ID=teaching-slam
bash course_support/k3_com260_kit/scripts/x86-ch09-ch10.bash start "$RUN_ID"
bash course_support/k3_com260_kit/scripts/x86-ch09-ch10.bash exec "$RUN_ID" bash
```

容器终端已经加载 `/workspace/install/setup.bash`。按代码块标明的主机分别启动、观测，完成后先 Ctrl+C 停止节点，再在 x86 宿主执行 `bash course_support/k3_com260_kit/scripts/x86-ch09-ch10.bash stop "$RUN_ID"`。每次只运行一个建图/定位系统，防止多个节点同时发布 `map→odom`。


```bash
# x86 课程容器终端 1
source /workspace/install/setup.bash
ros2 launch sim_demo sim_bringup.launch.py \
  world:="$(ros2 pkg prefix sim_demo)/share/sim_demo/worlds/museum.sdf" \
  sensor_profile:=teacher gui:=true rviz:=true
```

```bash
# COM260
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 bag record --use-sim-time -o /tmp/gmapping_input /scan /odom /tf /tf_static
# 运行约 10 秒后 Ctrl+C
ros2 bag info /tmp/gmapping_input
```

在独立 COM260 终端启动自动运动演示；先观察小车周边空间，按 Ctrl+C 停止：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run param_demo_cpp patrol_driver --ros-args -p linear_speed:=0.12 -p angular_speed:=0.45
```

此终端接替原 `drive:=true` 的自动运动：复用第6章 C++ `patrol_driver`，线速度0.12 m/s、角速度0.45 rad/s，替代原仿真入口默认5.0 m/s、1.5 rad/s。传感器/TF观察目标不变，实际速度与轨迹以里程计为准；不要同时运行键盘控制。

### 观察结果

检查 bag 中 `/scan` 和 `/odom` 的消息数量、时间范围与 frame_id；这些是粒子预测和观测更新的输入条件。

### 源码与边界

数据桥配置位于 `src_k3_com260_kit/sim_demo/config/bridge.yaml`，在线建图对照位于 `src_k3_com260_kit/slam_lab/src/slam_monitor.cpp`。

本例不运行 gmapping 算法，也不把 `slam_toolbox` 的地图作为 gmapping 结果。

学习材料：
- ROS Wiki —— gmapping 包文档与参数说明：https://wiki.ros.org/gmapping
- OpenSLAM —— gmapping 项目页（RBPF 网格建图）：https://openslam-org.github.io/gmapping.html
- SLAM Toolbox Wiki —— Steve Macenski 维护的参数与架构说明：https://github.com/SteveMacenski/slam_toolbox/wiki
- The Construct —— ROS 2 SLAM 课程（gmapping 到 slam_toolbox 的迁移）：https://www.theconstructsim.com/
- Robotics Back-End —— gMapping vs Cartographer 对比与建图实践：https://roboticsbackend.com/
