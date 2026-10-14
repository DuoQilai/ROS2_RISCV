# 第11章 ICP与PLICP扫描匹配

本章为 COM260 Kit 的 C++17 教学副本。课程概念、练习及数值要求沿用原章；实测状态与素材见[实验第10章](../lab_manuals_k3_com260_kit/ch10_lab.md)和[运行证据](../lab_manuals_k3_com260_kit/runtime_evidence.md#ch10)。算法片段展示相应数学步骤，不能代替完整 SLAM 系统。

数值示例和 ROS 已知输入已分别在 COM260 与 x86 构建运行。原始记录：[COM260数值](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-numerics.cast)、[COM260 ROS接口](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-ros.cast)、[真实仿真传感器](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-live-sensors.cast)。这些检查的输入和范围分开记录，不能据此宣称所有教学算法均在真实场景达到定位精度要求。

教学程序位于 [`slam_teaching_cpp`](../src_k3_com260_kit/slam_teaching_cpp/README.md)。COM260 终端先执行：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
```


> **课程**：ROS2 C++17 编程
> **章节**：第11章
> **课时**：2 课时（90 分钟）
> **教学方式**：讲授 + 演示

---

## 学习目标

本章学习目标包括：理解ICP扫描匹配算法的基本原理与数学推导，掌握PLICP（Point-to-Line ICP）算法及其优势，熟悉Gauss-Newton优化方法在扫描匹配中的应用，能够使用C++17实现基本的ICP和PLICP算法，并了解扫描匹配在ROS2 SLAM中的实际应用。

## 11.1 ICP扫描匹配算法

### 11.1.1 扫描匹配问题定义

扫描匹配 (Scan Matching) 是SLAM前端的核心步骤，目标是根据两帧激光点云之间的几何关系，求解机器人在这两帧之间的相对位姿变化。输入为参考帧点云 P = {p₁, p₂, ..., pₙ} 与当前帧点云 Q = {q₁, q₂, ..., qₘ}，输出为将 Q 映射到 P 坐标系的变换矩阵 T = (R, t)，其数学形式为：

```
T* = argmin Σ || p_i - T(q_i) ||²
```

其中 T(q_i) = R · q_i + t，R为旋转矩阵，t为平移向量。

### 11.1.2 ICP算法步骤

ICP (Iterative Closest Point) 通过迭代方式求解点云配准问题：

```
算法流程:
1. 初始化变换 T₀ (通常使用里程计或匀速模型)
2. 对Q中每个点，在P中寻找最近邻点
3. 计算使匹配点对距离和最小的变换 T
4. 应用T到Q，重复步骤2-3
5. 当收敛或达到最大迭代次数时停止
```

以下类/函数片段摘自 [`scan_matching.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/scan_matching.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

本例求 source→target 变换。增量 SVD 必须以当前已变换点求解，再累计变换；W=Σ(p−μp)(q−μq)ᵀ 时旋转为 VUᵀ。这里修正原例重复累计绝对变换及旋转方向错误。最近邻采用穷举的精确欧氏距离，结果语义与 KDTree 相同；教学小点云牺牲加速以减少依赖，课后 KDTree 加速练习保留。

```cpp
class ICP {
public:
  int max_iterations; double tolerance;
  explicit ICP(int iterations = 50, double tol = 1e-6) : max_iterations(iterations), tolerance(tol) {}
  MatchResult align(const Points & source, const Points & target,
    const Vector3d & initial = Vector3d::Zero()) const {
    if (source.empty() || target.empty()) { throw std::invalid_argument("ICP要求非空点云"); }
    Eigen::Matrix2d R = rotation(initial.z()); Vector2d t = initial.head<2>();
    MatchResult result;
    double previous = std::numeric_limits<double>::infinity();
    for (int step = 0; step < max_iterations; ++step) {
      Points transformed, matched;
      Vector2d mu_source = Vector2d::Zero(), mu_target = Vector2d::Zero();
      double error = 0.0;
      for (const auto & point : source) {
        const Vector2d p = R * point + t;
        const Vector2d q = target[nearest(p, target)];
        transformed.push_back(p); matched.push_back(q);
        mu_source += p; mu_target += q; error += (p - q).norm();
      }
      mu_source /= static_cast<double>(source.size()); mu_target /= static_cast<double>(source.size());
      Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
      for (std::size_t i = 0; i < source.size(); ++i) {
        W += (transformed[i] - mu_source) * (matched[i] - mu_target).transpose();
      }
      Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix2d V = svd.matrixV();
      Eigen::Matrix2d increment = V * svd.matrixU().transpose();
      if (increment.determinant() < 0) {
        V.col(1) *= -1; increment = V * svd.matrixU().transpose();
      }
      t = increment * t + mu_target - increment * mu_source;
      R = increment * R;
      error /= static_cast<double>(source.size()); result.errors.push_back(error);
      if (std::abs(previous - error) < tolerance) { break; }
      previous = error;
    }
    result.pose << t, std::atan2(R(1, 0), R(0, 0)); return result;
  }
};
```

### 11.1.3 ICP的数学推导

ICP的核心是最小化对应点之间的欧氏距离平方和：

```
E(R, t) = Σ || p_i - (R·q_i + t) ||²
```

**求解旋转：** 使用SVD分解

```
W = Σ (q_i - μ_q)(p_i - μ_p)ᵀ
W = U·Σ·Vᵀ
R = V·Uᵀ
```

**求解平移：** 旋转后计算

```
t = μ_p - R·μ_q
```

### 11.1.4 ICP的局限性

**局部最优：** ICP对初始位姿敏感，容易陷入局部最小值；**点云稀疏性：** 在结构简单环境中匹配质量差；**计算效率：** 最近邻搜索是O(n²)复杂度（使用KDTree可优化为O(n·log n)）；**对应关系误差：** 简单最近邻假设可能导致错误匹配。

以下类/函数片段摘自 [`analysis_examples.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/analysis_examples.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
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
```

## 11.2 PLICP点线ICP算法

### 11.2.1 PLICP原理

PLICP (Point-to-Line ICP) 是ICP的改进版本，使用点到直线的距离代替点到点的距离。这使得收敛速度更快、精度更高，尤其适用于结构化环境。

**目标函数：**

```
E(T) = Σ (n_iᵀ · (T(q_i) - p_i))²
```

其中 n_i 为目标点 p_i 处的法向量。

**点到线距离示意：**

```
      线(L)
        |
  q_i   |    p_i (参考点)
   ×----|----×
        |    ↑
        |    n_i (法向量)
  距离 = n_i · (T(q_i) - p_i)
```

### 11.2.2 法向量估计

法向量估计是PLICP的关键步骤：

以下类/函数片段摘自 [`scan_matching.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/scan_matching.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
class PointToLineICP {
public:
  int max_iterations; double tolerance;
  explicit PointToLineICP(int iterations = 20, double tol = 1e-6)
  : max_iterations(iterations), tolerance(tol) {}
  Points estimate_normals(const Points & points, int k = 5) const {
    if (points.size() < static_cast<std::size_t>(k + 1)) {
      throw std::invalid_argument("法向量估计要求至少 k+1 个点");
    }
    Points normals;
    for (std::size_t i = 0; i < points.size(); ++i) {
      std::vector<std::size_t> ids(points.size()); std::iota(ids.begin(), ids.end(), 0);
      std::stable_sort(ids.begin(), ids.end(), [&](std::size_t a, std::size_t b) {
        if (a == i) { return b != i; } if (b == i) { return false; }
        return (points[a] - points[i]).squaredNorm() < (points[b] - points[i]).squaredNorm();
      });
      Vector2d center = Vector2d::Zero();
      for (int j = 1; j <= k; ++j) { center += points[ids[j]]; } center /= k;
      Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
      for (int j = 1; j <= k; ++j) {
        Vector2d d = points[ids[j]] - center; covariance += d * d.transpose();
      }
      covariance /= k - 1;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
      Vector2d normal = solver.eigenvectors().col(0).normalized();
      if (normal.dot(points[i]) < 0) { normal *= -1; } normals.push_back(normal);
    }
    return normals;
  }
  MatchResult align(const Points & source, const Points & target,
    const Vector3d & initial = Vector3d::Zero()) const {
    if (source.empty()) { throw std::invalid_argument("PLICP要求非空源点云"); }
    MatchResult result; result.pose = initial;
    const auto normals = estimate_normals(target);
    double previous = std::numeric_limits<double>::infinity();
    for (int step = 0; step < max_iterations; ++step) {
      const double theta = result.pose.z();
      const Eigen::Matrix2d R = rotation(theta);
      Eigen::Matrix2d derivative; derivative << -std::sin(theta), -std::cos(theta), std::cos(theta), -std::sin(theta);
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero(); Vector3d b = Vector3d::Zero(); double error = 0.0;
      for (const auto & point : source) {
        const Vector2d transformed = R * point + result.pose.head<2>();
        const auto idx = nearest(transformed, target); const Vector2d & normal = normals[idx];
        const double residual = normal.dot(transformed - target[idx]);
        Vector3d J(normal.x(), normal.y(), normal.dot(derivative * point));
        H += J * J.transpose(); b += J * residual; error += residual * residual;
      }
      result.pose += (H + 1e-6 * Eigen::Matrix3d::Identity()).ldlt().solve(-b);
      result.pose.z() = wrap(result.pose.z());
      error /= static_cast<double>(source.size()); result.errors.push_back(error);
      if (std::abs(previous - error) < tolerance) { break; } previous = error;
    }
    return result;
  }
};
```

### 11.2.3 PLICP的数学推导

PLICP的目标函数（点到线距离的平方和）：

```
E(T) = Σ (n_iᵀ · (R·q_i + t - p_i))²
```

**Gauss-Newton优化：**

对变换参数 (x, y, θ) 进行线性化：

```
e_i(ξ + Δξ) ≈ e_i(ξ) + J_i · Δξ
```

其中雅可比 J_i：

```
J_i = [∂e_i/∂x, ∂e_i/∂y, ∂e_i/∂θ]
    = [n_i_x, n_i_y, n_i · (∂R/∂θ · q_i)]
```

Gauss-Newton更新：

```
Δξ = -(JᵀJ)⁻¹ · Jᵀ · e(ξ)
```

### 11.2.4 PLICP vs ICP对比

完整程序：[`icp_demo.cpp`](../src_k3_com260_kit/slam_teaching_cpp/src/icp_demo.cpp)。

保留原随机点云100点、种子42、旋转0.3 rad、平移(1.0,0.5)、噪声0.1，以及结构化墙面和PLICP输入。C++与NumPy的随机分布实现不同，不要求同种子产生逐点相同数据。生成变换是 target→source，评价真值必须取逆；误差不能直接与生成变换比较。ICP输出欧氏距离，PLICP输出点线距离平方，两者数值不能直接当作同一单位比较。

```cpp
#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include "slam_teaching_cpp/scan_matching.hpp"

int main() {
  using namespace slam_teaching;
  std::mt19937 random(42); std::uniform_real_distribution<double> uniform(0.0, 10.0);
  std::normal_distribution<double> noise(0.0, 0.1);
  Points target, source;
  const Eigen::Matrix2d true_R = rotation(0.3); const Vector2d true_t(1.0, 0.5);
  for (int i = 0; i < 100; ++i) {
    const double x = uniform(random), y = uniform(random); target.emplace_back(x, y);
  }
  for (const auto & point : target) {
    const double nx = noise(random), ny = noise(random);
    source.push_back(true_R * point + true_t + Vector2d(nx, ny));
  }
  auto result = ICP().align(source, target);
  Vector3d expected; expected << -true_R.transpose() * true_t, -0.3;
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "source_to_target expected=" << expected.transpose() << '\n';
  std::cout << "estimated=" << result.pose.transpose() << " iterations=" << result.errors.size()
            << " nearest_error=" << result.errors.back() << '\n';
  // 原章的结构化墙面比较：水平各100点、垂直各60点。
  Points walls, transformed;
  for (int i = 0; i < 100; ++i) { double x = -5.0 + 10.0 * i / 99.0; walls.emplace_back(x, -3.0); walls.emplace_back(x, 3.0); }
  for (int i = 0; i < 60; ++i) { double y = -3.0 + 6.0 * i / 59.0; walls.emplace_back(-5.0, y); walls.emplace_back(5.0, y); }
  noise = std::normal_distribution<double>(0.0, 0.05);
  for (const auto & point : walls) {
    const double nx = noise(random), ny = noise(random);
    transformed.push_back(rotation(0.15) * point + Vector2d(0.5, 0.3) + Vector2d(nx, ny));
  }
  auto start = std::chrono::steady_clock::now(); auto icp = ICP(50).align(transformed, walls);
  double icp_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  start = std::chrono::steady_clock::now(); auto plicp = PointToLineICP(20).align(transformed, walls);
  double plicp_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  Vector3d wall_expected; wall_expected << -rotation(0.15).transpose() * Vector2d(0.5, 0.3), -0.15;
  std::cout << "ICP pose=" << icp.pose.transpose() << " iterations=" << icp.errors.size() << " error=" << icp.errors.back() << " ms=" << icp_ms << '\n';
  std::cout << "PLICP pose=" << plicp.pose.transpose() << " iterations=" << plicp.errors.size() << " squared_line_error=" << plicp.errors.back() << " ms=" << plicp_ms << '\n';
  std::cout << "PLICP angle_error_deg=" << std::abs(wrap(plicp.pose.z() - wall_expected.z())) * 180.0 / std::acos(-1.0)
            << " translation_error=" << (plicp.pose.head<2>() - wall_expected.head<2>()).norm() << '\n';
  // 无噪声已知输入作为实现断言；带噪声实验结果单独报告。
  Points exact;
  for (const auto & point : target) { exact.push_back(rotation(0.03) * point + Vector2d(0.1, 0.05)); }
  const auto known = ICP().align(exact, target);
  Vector3d known_expected; known_expected << -rotation(0.03).transpose() * Vector2d(0.1, 0.05), -0.03;
  bool pass = (known.pose - known_expected).norm() < 1e-6 && plicp.pose.allFinite();
  std::cout << "known_input_error=" << (known.pose - known_expected).norm() << " pass=" << std::boolalpha << pass << '\n';
  return pass ? 0 : 1;
}
```

## 11.3 Gauss-Newton优化方法

COM260 的一次 Debug 数值运行中，100点带噪实验得到 source→target 位姿 `(-0.607006, -0.617133, -0.151175)`，而对应真值为 `(-1.103097, -0.182148, -0.300000)`：该组数据陷入局部极小值，21次迭代和0.453940 m最近邻误差并不代表配准正确。结构化墙面实验中，PLICP为5次迭代、角度误差0.018187°、平移误差0.004886 m。程序的`pass`只断言无噪声已知输入和结果有限性，不能覆盖带噪实验的收敛精度；完整输出按[运行证据](../lab_manuals_k3_com260_kit/runtime_evidence.md#ch10)登记。


### 11.3.1 Gauss-Newton原理

Gauss-Newton法是一种用于非线性最小二乘问题的迭代优化算法，特别适用于扫描匹配。

**问题形式：**

```
min F(x) = Σ f_i(x)² = ||f(x)||²
```

**迭代更新：**

```
x_{k+1} = x_k - (JᵀJ)⁻¹ · Jᵀ · f(x_k)
```

其中 J 为雅可比矩阵，f_i(x) 为残差函数。

### 11.3.2 在扫描匹配中的应用

以下类/函数片段摘自 [`analysis_examples.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/analysis_examples.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

保留10次、1e−4收敛阈值和逐点正规方程。坐标直接以栅格为单位，调用者须提供同尺寸的x/y梯度；边界检查包含x+1/y+1，避免原例最后一行/列访问越界。

```cpp
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
```

### 11.3.3 Gauss-Newton vs 其他优化方法

| 方法 | 收敛速度 | 实现难度 | 需二阶导 | 适用场景 |
|------|---------|---------|---------|---------|
| 梯度下降 | 慢 | 易 | 否 | 远离最优值 |
| Gauss-Newton | 快 | 中 | 否 | 残差较小时 |
| Levenberg-Marquardt | 快 | 中 | 否 | 通用推荐 |
| 牛顿法 | 最快 | 难 | 是 | 小规模问题 |

### 11.3.4 官方要点——Cartographer 的分层匹配策略：粗匹配 + 精细优化

Google Cartographer 官方文档提出的「先粗后精」两阶段策略，是理解本章 11.3 Gauss-Newton 方法的最佳延伸：第一阶段用 Real-Time Correlative Scan Matching（实时相关性扫描匹配）在预先生成的搜索窗口内暴力枚举候选位姿，用相关性打分选出较优初值；第二阶段再用 Ceres 求解器做精细优化——其本质正是本章 11.3.1 的 Gauss-Newton（残差平方和最小化）并支持损失函数加权，对应本章 11.5.2 的鲁棒匹配思想与 11.3.3 中 LM 法对 GN 的改进（在正规方程对角线加阻尼项，本章 11.3.2 代码中的 `+ I·1e-6` 即最小阻尼）。

官方文档明确强调：粗匹配窗口大小（`linear_search_window`、`angular_search_window`）决定初值容错范围，窗口越大越能容忍里程计漂移，但计算量线性增长；窗口过小则粗匹配直接落在错误极值附近，精细优化无法挽回——这正是本章 11.1.4 所述「初始位姿敏感、易陷局部最优」的工程对策。

## 11.4 ROS2中的扫描匹配应用

### 11.4.1 slam_toolbox中的扫描匹配

slam_toolbox内部使用Ceres Solver进行优化。原示例中的translation_weight=10.0、rotation_weight=40.0、occupied_space_weight=20.0属于Cartographer式权重对照，不是本版slam_toolbox的参数；不能放入双层ceres_scan_matcher后声称已生效。以下使用实际Ceres求解器参数：

```yaml
# slam_toolbox参数中的扫描匹配配置
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_loss_function: None
    minimum_time_interval: 0.5        # 最小帧间隔
    minimum_travel_distance: 0.3      # 最小移动距离触发放射匹配
    resolution: 0.05                  # 地图分辨率
    max_laser_range: 8.0             # 最大激光范围
```

### 11.4.2 ROS2扫描匹配节点示例

完整程序：[`scan_matcher.cpp`](../src_k3_com260_kit/slam_teaching_cpp/src/scan_matcher.cpp)。

保留第6帧建参考、0.5 s定时、误差0.01更新参考。参考帧滑动后初值应使用参考时刻到当前的相对里程计，不能把全局绝对里程计直接作为相邻扫描初值。此节点只输出日志，原文 /scan_matcher/matched_pose 并未发布，不把不存在的话题列为验收入口。

```cpp
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "slam_teaching_cpp/scan_matching.hpp"
class ScanMatcherNode : public rclcpp::Node {
public:
  ScanMatcherNode() : Node("scan_matcher") {
    scan_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        latest_.clear();
        for (std::size_t i=0;i<msg->ranges.size();++i) {
          double r=msg->ranges[i],a=msg->angle_min+i*msg->angle_increment;
          if (std::isfinite(r) && r>msg->range_min && r<msg->range_max) { latest_.emplace_back(r*std::cos(a),r*std::sin(a)); }
        }
        if (++scan_count_>5 && reference_.empty() && latest_.size()>=6) {
          reference_=latest_; reference_odom_=odom_pose_;
          RCLCPP_INFO(get_logger(),"参考扫描已初始化: %zu点",reference_.size());
        }
      });
    odom_=create_subscription<nav_msgs::msg::Odometry>("/odom",10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & p=msg->pose.pose; const auto & q=p.orientation;
        odom_pose_ << p.position.x,p.position.y,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      });
    timer_=create_wall_timer(std::chrono::milliseconds(500),[this](){
      if (latest_.size()<6 || reference_.size()<6) { return; }
      slam_teaching::Vector3d initial;
      initial << slam_teaching::rotation(reference_odom_.z()).transpose()*(odom_pose_.head<2>()-reference_odom_.head<2>()),
        slam_teaching::wrap(odom_pose_.z()-reference_odom_.z());
      const auto result=slam_teaching::PointToLineICP().align(latest_,reference_,initial);
      RCLCPP_INFO(get_logger(),"扫描匹配: 位姿=(%.3f, %.3f, %.1f°) 误差=%.6f 迭代=%zu",
        result.pose.x(),result.pose.y(),result.pose.z()*180/std::acos(-1.0),result.errors.back(),result.errors.size());
      if (result.errors.back()<0.01) { reference_=latest_; reference_odom_=odom_pose_; }
    });
  }
private:
  slam_teaching::Points latest_,reference_; std::size_t scan_count_=0;
  slam_teaching::Vector3d odom_pose_=slam_teaching::Vector3d::Zero(),reference_odom_=slam_teaching::Vector3d::Zero();
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<ScanMatcherNode>()); rclcpp::shutdown();
}
```

### 11.4.3 扫描匹配性能评估

```bash
# 使用rosbag测试扫描匹配
ros2 bag play slam_data_bag --loop

# 查看扫描匹配输出
# 查看运行 scan_matcher 的终端日志（教学节点只输出日志）

# 检查TF拓扑连通性；配准精度需另与独立真值或已知变换比较
ros2 run tf2_tools view_frames
```

### 11.4.4 扫描匹配调优建议

**点云预处理：** 体素滤波降采样（voxel_filter_size=0.05）、去除离群点（半径滤波）、提取特征点（角点、边缘点）。

**初始位姿估计：** 使用里程计作为初值、匀速运动模型外推、IMU预积分辅助。

**匹配参数调整：**

   ```
   场景类型      → 平移权重   → 旋转权重
   结构化走廊   → 10.0      → 40.0
   开阔广场     → 20.0      → 20.0
   狭窄通道     → 5.0       → 60.0
   ```

**收敛条件：** 最大迭代次数10-20次、误差变化阈值1e-4 - 1e-6、变换增量阈值0.001m / 0.001rad。

### 11.4.5 官方要点——扫描匹配在 SLAM 前端中的角色：slam_toolbox 的官方实现

SLAM Toolbox Wiki 把扫描匹配定位为「前端核心」：它负责把每一帧新激光与现有地图（或前一帧）配准，输出相对位姿增量，供后端图优化消费。本章11.4.1展示求解器参数。Cartographer的平移、旋转和占据空间权重用于理解不同残差的作用，不能作为slam_toolbox同名配置直接套用；实际可用参数以安装包及本课程mapper_params_online_async.yaml为准。

另外两个触发参数值得注意：`minimum_time_interval`（两次匹配的最小时间间隔）与 `minimum_travel_distance`（两次匹配的最小移动距离，本章 11.4.1 中为 0.3m）。它们保证机器人静止或原地旋转时不会反复触发代价高昂的匹配计算，是工程化的节流手段。

## 11.5 实际案例分析

### 11.5.1 走廊环境扫描匹配

走廊环境是扫描匹配的典型场景，具有明显的直线特征：

以下类/函数片段摘自 [`analysis_examples.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/analysis_examples.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

保留2 m宽、10 m长、每侧200点、位移(0.5,0,0.05)、噪声0.02以及ICP30次/PLICP15次。实际收敛次数和耗时以输出为准，不承诺固定倍数加速。

```cpp
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
```

### 11.5.2 动态环境中的鲁棒匹配

以下类/函数片段摘自 [`analysis_examples.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/analysis_examples.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

修正原加权SVD使用source与自身乘积的问题：互协方差必须为Σwi(pi−μp)(qi−μq)ᵀ；使用当前已变换源点求增量，角度按旋转矩阵正确累计。Huber阈值0.5、默认30次保持不变。

```cpp
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
```

### 11.5.3 官方要点——初值估计与数据关联的工程习惯

国际课程与 Wiki 反复强调初值（Initial Guess）质量决定扫描匹配成败。官方实践优先选用：里程计推算（本章 11.4.2 的 `odom_callback` 正是如此）、IMU 航向（有 IMU 时旋转初值更可靠）、匀速运动模型外推（高频场景下比里程计更平滑）。Nav2 与 slam_toolbox 中的 `scan matching score`（匹配得分）阈值便是用来判断本次匹配是否可信：得分过低时宁可信任里程计跳过本帧，也不要把错误增量送入后端。

对于动态环境，Robotics Back-End 的文章与本书 11.5.2 的 `RobustScanMatcher` 思路一致：用 Huber / Cauchy 损失函数给大残差降权。工程上另一常用手段是「角度与距离范围过滤」——只保留 `range_min ~ max_laser_range` 内命中率稳定的射线，避免动态行人腿部的摆动点污染匹配。

### 11.5.4 官方要点——退化场景与调参纪律

退化（Degeneracy）是扫描匹配最危险的失效模式：在长走廊或如本章 11.5.1 所示对称环境中，沿走廊方向移动时约束被「吸收」，算法收敛却完全错位。Cartographer 与 slam_toolbox 文档共同建议：配准结果要结合里程计一致性检查；走廊场景优先提高旋转权重、限制最大匹配距离，并依赖回环检测在全局层面纠偏。

调参纪律方面，两大官方项目一致推荐：先 `ros2 bag record --use-sim-time /scan /odom /tf` 录制数据，以`online_sync_launch.py`配合`ros2 bag play --clock`离线反复试参数，确认匹配得分稳定后再上在线模式；每次只改一个参数并记录建图效果。建议读者按本章练习第 5 题，在仓库的 `mapper_params_online_async.yaml`（`src_k3_com260_kit/slam_lab/config/`）基础上逐项对比。

## 课后练习

1. **推导题:** 推导ICP算法中基于SVD的最优变换求解过程，说明为什么SVD分解可以求得旋转矩阵的最优解。

2. **编程题:** 使用C++17实现一个完整的ICP算法，能够处理二维点云的配准，支持KDTree加速最近邻搜索，并绘制配准前后对比图。

3. **分析题:** 比较ICP和PLICP的异同点，从目标函数、收敛速度、对噪声的鲁棒性三个方面进行分析。

4. **推导题:** 推导PLICP中Gauss-Newton优化的雅可比矩阵，给出完整的数学推导过程。

5. **配置题:** 在slam_toolbox中调整扫描匹配参数（平移权重、旋转权重、最小移动距离等），说明不同参数对建图质量的影响。

6. **设计题:** 设计一个融合里程计和激光扫描匹配的位姿估计系统，说明在何种情况下应该更信任里程计，在何种情况下应该更信任扫描匹配。

---

## 仿真结合实例（当前仓库）：采集相邻激光帧作为扫描匹配输入

### 目标与知识点对应

ICP/PLICP 需要连续的激光扫描和位姿初值。本实例先从 Gazebo 获得稳定的 `/scan`、`/odom`、`/tf` 输入，并验证数据可供外部匹配器消费。

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
# 另一个终端记录短时间扫描和里程计
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 bag record --use-sim-time -o /tmp/scan_match_input /scan /odom /tf /tf_static
# 约 10 秒后 Ctrl+C
ros2 bag info /tmp/scan_match_input
```

在独立 COM260 终端启动自动运动演示；先观察小车周边空间，按 Ctrl+C 停止：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run param_demo_cpp patrol_driver --ros-args -p linear_speed:=0.12 -p angular_speed:=0.45
```

此终端接替原 `drive:=true` 的自动运动：复用第6章 C++ `patrol_driver`，线速度0.12 m/s、角速度0.45 rad/s，替代原仿真入口默认5.0 m/s、1.5 rad/s。传感器/TF观察目标不变，实际速度与轨迹以里程计为准；不要同时运行键盘控制。

### 观察结果

Bag 中同时包含扫描帧、里程计和 TF，可用于比较相邻帧的初始位姿与匹配增量；在 RViz 中显示 `LaserScan`，机器人运动时扫描相对环境发生变化。

### 源码与边界

仿真桥位于 `src_k3_com260_kit/sim_demo/config/bridge.yaml`，SLAM 输入配置位于 `src_k3_com260_kit/slam_lab/config/mapper_params_online_async.yaml`。C++教学求解器位于`slam_teaching_cpp/scan_matching.hpp`；先完成输入数据采集，再运行`ros2 run slam_teaching_cpp scan_matcher --ros-args -p use_sim_time:=true`，记录配准输出。数值已知输入与仿真配准结果分别验收。

---

学习材料：
- SLAM Toolbox Wiki —— 扫描匹配参数与工作流：https://github.com/SteveMacenski/slam_toolbox/wiki
- Google Cartographer 官方文档：https://google-cartographer-ros.readthedocs.io/
- The Construct —— ROS 2 与 SLAM 相关课程：https://www.theconstructsim.com/
- Robotics Back-End —— SLAM 技术文章：https://roboticsbackend.com/
