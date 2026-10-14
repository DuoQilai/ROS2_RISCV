# 第10章 SLAM基本概念与贝叶斯框架

本章为 COM260 Kit 的 C++17 教学副本。课程概念、练习及数值要求沿用原章；实测状态与素材见[实验第10章](../lab_manuals_k3_com260_kit/ch10_lab.md)和[运行证据](../lab_manuals_k3_com260_kit/runtime_evidence.md#ch10)。算法片段展示相应数学步骤，不能代替完整 SLAM 系统。

数值示例和 ROS 已知输入已分别在 COM260 与 x86 构建运行。原始记录：[COM260数值](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-numerics.cast)、[COM260 ROS接口](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-ros.cast)、[真实仿真传感器](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-live-sensors.cast)。这些检查的输入和范围分开记录，不能据此宣称所有教学算法均在真实场景达到定位精度要求。

教学程序位于 [`slam_teaching_cpp`](../src_k3_com260_kit/slam_teaching_cpp/README.md)。COM260 终端先执行：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
```


> **课程**：ROS2 C++17 编程
> **章节**：第10章
> **课时**：2 课时（90 分钟）
> **教学方式**：讲授 + 演示

---

## 学习目标

本章学习目标包括：理解 SLAM 问题的数学定义与核心挑战，掌握贝叶斯滤波框架在 SLAM 中的应用，熟悉 SLAM 系统中的传感器模型与 TF 坐标变换体系，了解 SLAM 方法的分类及其适用场景，并能够推导基本贝叶斯滤波公式。

## 10.1 SLAM问题定义

### 10.1.1 SLAM问题的起源与意义

SLAM (Simultaneous Localization and Mapping) 是移动机器人在未知环境中同时进行自身定位和环境地图构建的核心技术。这是机器人领域公认的基础性难题，被誉为"机器人学的圣杯"之一。

SLAM需要同时解决两个相互耦合的问题：**定位**——机器人根据传感器数据确定自身在环境中的位姿 (x, y, θ)；**建图**——建立周围环境的栅格地图，用于后续导航和避障。定位与建图之间存在鸡生蛋蛋生鸡的关系：要精确定位，需要知道环境地图；要构建精确地图，需要知道机器人位姿。SLAM的核心挑战在于打破这种耦合关系，在没有任何先验信息的情况下同时完成两项任务。

### 10.1.2 SLAM问题的数学形式化

SLAM的概率公式化表达为：

```
p(x_{1:t}, m | z_{1:t}, u_{1:t-1})
```

其中 `x_{1:t} = {x_1, x_2, ..., x_t}` 表示机器人从时刻1到t的完整位姿序列，`m` 表示环境地图（栅格地图或特征地图），`z_{1:t} = {z_1, z_2, ..., z_t}` 表示从时刻1到t的所有观测数据，`u_{1:t-1} = {u_1, u_2, ..., u_{t-1}}` 表示从时刻1到t-1的所有控制输入。该后验概率的意义是：在给定所有观测数据和控制输入的情况下，同时估计机器人路径和地图的联合概率分布。

### 10.1.3 SLAM问题的分解

SLAM问题可以分解为两个递归步骤。**预测步骤（运动模型）** 如下：

```
p(x_t, m | z_{1:t-1}, u_{1:t-1}) =
∫ p(x_t | x_{t-1}, u_t) · p(x_{t-1}, m | z_{1:t-1}, u_{1:t-2}) dx_{t-1}
```

**更新步骤（观测模型）** 如下：

```
p(x_t, m | z_{1:t}, u_{1:t-1}) =
η · p(z_t | x_t, m) · p(x_t, m | z_{1:t-1}, u_{1:t-1})
```

其中 η 为归一化常数，确保概率和为1。

## 10.2 贝叶斯滤波框架

### 10.2.1 贝叶斯滤波基本原理

贝叶斯滤波是SLAM问题的核心数学工具，它提供了根据观测数据递归估计状态的概率框架。基本思想是：利用状态转移模型预测当前状态，然后利用观测模型更新预测。

以下类/函数片段摘自 [`bayes.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/bayes.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

本节保留原例三维位姿、dt=0.1、初始协方差0.1及单位观测模型；只演示滤波步骤。

```cpp
class BayesianFilter {
public:
  Vector3d belief_mean = Vector3d::Zero();
  Eigen::Matrix3d belief_cov = Eigen::Matrix3d::Identity() * 0.1;
  void predict(const Vector2d & control, const Vector3d & motion_noise) {
    const double theta = belief_mean.z();
    belief_mean += Vector3d(control.x() * std::cos(theta) * 0.1,
      control.x() * std::sin(theta) * 0.1, control.y() * 0.1);
    belief_cov += motion_noise.asDiagonal();
  }
  void update(const Vector3d & observation, const Vector3d & obs_noise) {
    Eigen::Matrix3d S = belief_cov + obs_noise.asDiagonal().toDenseMatrix();
    Eigen::Matrix3d K = belief_cov * S.inverse();
    belief_mean += K * (observation - belief_mean);
    belief_cov = (Eigen::Matrix3d::Identity() - K) * belief_cov;
  }
};
```

### 10.2.2 马尔可夫假设

贝叶斯滤波有效的前提是马尔可夫假设：**状态完备性**——当前状态包含了预测未来所需的所有信息；**观测独立性**——给定当前状态，当前观测与过去观测独立。在SLAM中，这意味着机器人未来位姿只取决于当前位姿和控制输入，当前传感器观测只取决于当前位姿和地图。

### 10.2.3 贝叶斯滤波的SLAM实现

以下类/函数片段摘自 [`bayes.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/bayes.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
class SLAMBayesianFilter {
public:
  VectorXd mean;
  MatrixXd cov;
  std::vector<bool> landmarks_seen;
  explicit SLAMBayesianFilter(int num_landmarks = 100)
  : mean(VectorXd::Zero(3 + 2 * num_landmarks)),
    cov(MatrixXd::Identity(mean.size(), mean.size()) * 0.1),
    landmarks_seen(num_landmarks, false) {}
  void predict(const Vector2d & odom, const Vector3d & noise) {
    const double v = odom.x(), theta = mean[2], dt = 0.1;
    mean.head<3>() += Vector3d(v * std::cos(theta) * dt,
      v * std::sin(theta) * dt, odom.y() * dt);
    MatrixXd G = MatrixXd::Identity(mean.size(), mean.size());
    G(0, 2) = -v * std::sin(theta) * dt;
    G(1, 2) = v * std::cos(theta) * dt;
    cov = G * cov * G.transpose();
    cov.topLeftCorner<3, 3>() += noise.asDiagonal();
  }
  void update_with_landmark(const Vector2d & z, int id, const Vector2d & noise) {
    const int idx = 3 + 2 * id;
    if (!landmarks_seen.at(id)) {
      mean.segment<2>(idx) = mean.head<2>() + z.x() *
        Vector2d(std::cos(mean[2] + z.y()), std::sin(mean[2] + z.y()));
      landmarks_seen[id] = true;
      return;
    }
    Vector2d d = mean.segment<2>(idx) - mean.head<2>();
    const double q = d.squaredNorm(), r = std::sqrt(q);
    if (q == 0.0) { throw std::invalid_argument("路标与机器人重合，观测雅可比未定义"); }
    MatrixXd H = MatrixXd::Zero(2, mean.size());
    H(0, 0) = -d.x() / r; H(0, 1) = -d.y() / r;
    H(0, idx) = d.x() / r; H(0, idx + 1) = d.y() / r;
    H(1, 0) = d.y() / q; H(1, 1) = -d.x() / q; H(1, 2) = -1.0;
    H(1, idx) = -d.y() / q; H(1, idx + 1) = d.x() / q;
    Vector2d innovation(z.x() - r, wrap(z.y() - std::atan2(d.y(), d.x()) + mean[2]));
    Eigen::Matrix2d S = H * cov * H.transpose() + noise.asDiagonal().toDenseMatrix();
    MatrixXd K = cov * H.transpose() * S.inverse();
    mean += K * innovation;
    cov = (MatrixXd::Identity(mean.size(), mean.size()) - K * H) * cov;
  }
};
```

### 10.2.4 贝叶斯滤波的局限性

纯贝叶斯滤波在SLAM中面临以下挑战：**高维状态空间**——包含地图点后，状态维度呈线性增长；**非线性问题**——运动模型和观测模型高度非线性；**数据关联**——需要正确匹配观测与地图点；**计算复杂度**——协方差矩阵更新为 O(n²) 复杂度。这些局限性催生了GraphSLAM、粒子滤波SLAM等更先进的SLAM方法。

## 10.3 传感器与坐标变换

### 10.3.1 SLAM常用传感器

**2D激光雷达 (LaserScan)** 是室内导航主要传感器，提供270°-360°范围内距离测量，发布 `sensor_msgs/LaserScan` 话题，典型参数为角度分辨率0.25°-1°、最大测距8-30m。**3D激光雷达 (PointCloud2)** 用于室外环境和大场景SLAM，发布 `sensor_msgs/PointCloud2` 话题，代表产品为 Velodyne VLP-16、Ouster OS系列。**深度相机 (RGB-D)** 同时提供RGB图像和深度图，代表产品为 Intel RealSense D415/D435、Microsoft Kinect，适用于视觉SLAM和室内精细建图。**IMU (惯性测量单元)** 提供加速度和角速度测量，工作频率高（100-1000Hz）但存在漂移，通常与激光/视觉融合使用。**里程计 (Odometry)** 通过轮式编码器估算机器人位移，短时间内精度高，长时间存在累积误差。

完整程序：[`sensor_monitor.cpp`](../src_k3_com260_kit/slam_teaching_cpp/src/sensor_monitor.cpp)。



```cpp
#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

class SensorMonitor : public rclcpp::Node {
public:
  SensorMonitor() : Node("sensor_monitor") {
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        ++scan_count_;
        const auto valid = std::count_if(msg->ranges.begin(), msg->ranges.end(),
          [msg](float r) { return r > 0 && r < msg->range_max; });
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "激光: %zu条射线, %td个有效点, 范围[%.2f, %.2f]",
          msg->ranges.size(), valid, msg->range_min, msg->range_max);
      });
    imu_ = create_subscription<sensor_msgs::msg::Imu>("/imu", 10,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        ++imu_count_;
        const auto & a = msg->linear_acceleration; const auto & g = msg->angular_velocity;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "IMU: acc=(%.2f,%.2f,%.2f) gyro=(%.2f,%.2f,%.2f)", a.x, a.y, a.z, g.x, g.y, g.z);
      });
    odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        ++odom_count_;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "里程计: pos=(%.2f,%.2f) vel=(%.2fm/s)",
          msg->pose.pose.position.x, msg->pose.pose.position.y, msg->twist.twist.linear.x);
      });
  }
private:
  std::size_t scan_count_ = 0, imu_count_ = 0, odom_count_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<SensorMonitor>()); rclcpp::shutdown();
}
```

### 10.3.2 TF坐标变换体系

SLAM系统依赖TF坐标变换树来关联不同传感器和本体坐标系：

```
map → odom → base_footprint → base_link → laser
```

各坐标系含义：**map（地图坐标系）** 是全局世界坐标系，为SLAM构建地图所在的固定参考系；**odom（里程计坐标系）** 是机器人起始位置，由里程计连续更新，局部精度高；**base_footprint（机器人足迹坐标系）** 是机器人在地面的投影，常作为机器人本体坐标系；**base_link（机器人基座坐标系）** 是机器人本体的中心坐标系；**laser（激光雷达坐标系）** 是激光传感器的坐标系，通常位于机器人顶部。map→odom的变换由SLAM/AMCL持续更新，以消除里程计累积误差。

### 10.3.3 TF树在SLAM中的作用

完整程序：[`tf_monitor.cpp`](../src_k3_com260_kit/slam_teaching_cpp/src/tf_monitor.cpp)。

can_transform_check 返回 canTransform 的真实布尔值，修正原例无异常时总是返回 True 的问题。原教学laser名称改为Burger模型实际laser_link；点(1,0,0)及map目标坐标系不变。

```cpp
#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TFMonitor : public rclcpp::Node {
public:
  TFMonitor() : Node("tf_monitor"), buffer_(get_clock()), listener_(buffer_) {
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { query_tf(); });
  }
  bool can_transform_check(const std::string & source, const std::string & target) {
    return buffer_.canTransform(target, source, tf2::TimePointZero, tf2::durationFromSec(0.1));
  }
private:
  void query_tf() {
    try {
      const auto trans = buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      RCLCPP_INFO(get_logger(), "map→base_link: (%.3f, %.3f)",
        trans.transform.translation.x, trans.transform.translation.y);
      geometry_msgs::msg::PointStamped point;
      point.header.frame_id = "laser_link"; point.point.x = 1.0;
      const auto mapped = buffer_.transform(point, "map");
      RCLCPP_INFO(get_logger(), "激光点(1,0,0)在map下: (%.3f, %.3f)", mapped.point.x, mapped.point.y);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "TF查询失败: %s", e.what());
    }
  }
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<TFMonitor>()); rclcpp::shutdown();
}
```

### 10.3.4 传感器标定与时间同步

SLAM系统中多传感器的时间同步至关重要：

```bash
# 检查传感器话题频率
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /imu

# 检查传感器时间戳
ros2 topic echo /scan --once | grep stamp
ros2 topic echo /odom --once | grep stamp

# 查看TF树
ros2 run tf2_tools view_frames

# 查看TF广播关系
ros2 run tf2_ros tf2_monitor
```

### 10.3.5 官方要点——传感器模型与 TF 体系在官方实践中的落地

SLAM Toolbox Wiki 与 Nav2 文档对坐标体系的说明与本节 10.3.2 完全一致：`map → odom → base_footprint → base_link → laser`。slam_toolbox 的 `base_frame`、`odom_frame`、`map_frame` 三个参数必须与 TF 树中的实际 frame 名逐一对应，不匹配或 TF 超时（`transform_tolerance` 设置过小）时建图会直接中断。

时间同步（本节 10.3.4）在官方工作流中有两条铁律：其一，仿真与 bag 重放一律开 `use_sim_time`；其二，录数据前先 `ros2 topic hz` 检查各传感器频率，避免激光帧率忽高忽低导致匹配抖动。Nav2 的 AMCL 文档也要求里程计与激光话题时间戳一致，否则协方差更新会失效。

## 10.4 SLAM方法分类

### 10.4.1 基于滤波的方法

基于滤波的SLAM方法使用概率滤波器（EKF、粒子滤波等）在线递归估计机器人位姿和地图。代表算法如下表所示：

| 算法 | 滤波器类型 | 特点 |
|------|-----------|------|
| EKF-SLAM | 扩展卡尔曼滤波 | 最早的方法，计算O(n²) |
| FastSLAM 1.0/2.0 | Rao-Blackwellized粒子滤波 | 粒子表示路径，EKF维护地图 |
| gmapping | RBPF + 自适应提议分布 | 广泛使用的2D SLAM |
| UKF-SLAM | 无迹卡尔曼滤波 | 处理强非线性 |

其优点是：在线计算、实时性强，自然处理不确定性，易于嵌入控制回路。缺点则是：存在线性化误差（EKF）、粒子退化问题（粒子滤波），且对大规模环境扩展性有限。

### 10.4.2 基于优化的方法

基于优化的方法将SLAM构建为图优化问题，通过最小化误差函数求解。代表算法如下表所示：

| 算法 | 优化方法 | 特点 |
|------|----------|------|
| GraphSLAM | 稀疏矩阵分解 | 全图优化，离线批处理 |
| Cartographer | Ceres Solver | 子图和回环检测，Google开源 |
| SLAM Toolbox | Ceres Solver | ROS2官方2D SLAM工具 |
| KartoSLAM | SPA (Sparse Pose Adjustment) | 基于图优化 |

图优化框架中，节点 (Vertex) 包括机器人位姿 x_t 与路标点 l_j，边 (Edge) 包括里程计约束（连续位姿间）、观测约束（位姿-路标点）与回环约束（非连续位姿间）：

```
节点 (Vertex):    边 (Edge):
- 机器人位姿 x_t   - 里程计约束 (连续位姿间)
- 路标点 l_j      - 观测约束 (位姿-路标点)
                  - 回环约束 (非连续位姿间)
```

以下类/函数片段摘自 [`bayes.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/bayes.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

本例仅实现原文线性系统实际处理的里程计边。原文虽然保存路标与观测边，优化器没有使用它们，不构成完整 GraphSLAM。残差 e=z−(xj−xi) 的雅可比为 Ji=I、Jj=−I，因此正规方程应求 HΔ=−b；这里修正原例更新符号。

```cpp
class GraphSLAM {
public:
  struct Constraint { int i, j; Vector3d measurement; Eigen::Matrix3d information; };
  std::vector<Vector3d> poses;
  std::vector<Constraint> constraints;
  void add_odometry_constraint(int i, int j, const Vector3d & measurement,
    const Eigen::Matrix3d & information) { constraints.push_back({i, j, measurement, information}); }
  void optimize() {
    if (poses.empty()) { return; }
    const int n = static_cast<int>(3 * poses.size());
    MatrixXd H = MatrixXd::Zero(n, n);
    VectorXd b = VectorXd::Zero(n);
    for (const auto & edge : constraints) {
      // 简化的全局坐标加性约束，不是完整 SE(2) 相对位姿模型。
      const Vector3d error = edge.measurement - (poses.at(edge.j) - poses.at(edge.i));
      const int i = 3 * edge.i, j = 3 * edge.j;
      H.block<3, 3>(i, i) += edge.information;
      H.block<3, 3>(j, j) += edge.information;
      H.block<3, 3>(i, j) -= edge.information;
      H.block<3, 3>(j, i) -= edge.information;
      b.segment<3>(i) += edge.information * error;
      b.segment<3>(j) -= edge.information * error;
    }
    H.topRows(3).setZero(); H.leftCols(3).setZero();
    H.topLeftCorner<3, 3>().setIdentity(); b.head<3>().setZero();
    VectorXd delta = (H + 1e-6 * MatrixXd::Identity(n, n)).ldlt().solve(-b);
    for (std::size_t i = 0; i < poses.size(); ++i) { poses[i] += delta.segment<3>(3 * i); }
  }
};
```

### 10.4.3 基于视觉的方法

视觉SLAM使用相机作为主要传感器，提取图像特征进行定位和建图。代表算法包括：**ORB-SLAM2/3**——特征法视觉SLAM，采用三线程架构；**DSO (Direct Sparse Odometry)**——直接法，利用像素亮度；**SVO (Semi-direct Visual Odometry)**——半直接法；**VINS-Mono**——视觉惯性融合SLAM。

### 10.4.4 混合方法

现代SLAM系统往往结合多种方法的优势：

| 方法 | 代表 | 前端 | 后端 |
|------|------|------|------|
| 激光+IMU融合 | Cartographer | 扫描匹配 | 图优化 |
| 视觉+惯性融合 | VINS-Fusion | 特征跟踪 | 滑窗BA |
| 激光+视觉融合 | LVI-SAM | 多传感器前端 | 因子图优化 |

### 10.4.5 方法选择指南

选择SLAM算法时需考虑环境类型、传感器类型、里程计/IMU 可用性与实时性要求等因素：

以下类/函数片段摘自 [`bayes.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/bayes.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
inline std::string suggest_slam_algorithm(const std::string & environment = "indoor",
  bool has_odom = true, bool has_imu = false, const std::string & sensor_type = "lidar2d",
  const std::string & precision_requirement = "high", bool realtime = true) {
  (void)precision_requirement; (void)realtime;  // 原示例未使用这两个选型因素。
  if (sensor_type == "lidar2d") {
    if (environment == "indoor" && !has_odom) { return "Hector-SLAM (不依赖里程计)"; }
    if (environment == "indoor" && has_odom) { return "slam_toolbox (ROS2推荐) / gmapping"; }
    if (environment == "large_scale") { return "Cartographer (大场景图优化)"; }
  }
  if (sensor_type == "lidar3d") { return "Cartographer 3D / A-LOAM"; }
  if (sensor_type == "camera") { return has_imu ? "VINS-Fusion / ORB-SLAM3" : "ORB-SLAM2 / DSO"; }
  if (sensor_type == "mixed") { return "LVI-SAM / R3LIVE"; }
  return "slam_toolbox (默认推荐)";
}
```

### 10.4.6 官方要点——从贝叶斯滤波到主流 SLAM 方法的系统对照

Cartographer 官方文档将其算法拆为「前端 + 后端」两阶段：前端执行局部 SLAM，用 Ceres 求解器做扫描匹配，把激光帧拼接到局部子图（submap）上；后端执行全局 SLAM，在子图之间检测回环并做全局图优化。这正是本节 10.4.2 图优化框架「节点 = 机器人位姿、边 = 里程计/观测/回环约束」的工业级实现，也与 10.5.4 信息形式（稀疏信息矩阵 + 迭代求解）的数学动机直接对应。

SLAM Toolbox Wiki 同时指出：gmapping 属于 Rao-Blackwellized 粒子滤波（RBPF）家族，对应本节 10.4.1 的 FastSLAM 思想——粒子表示机器人轨迹、每个粒子独立维护地图，利用 10.5.3 的条件独立性简化；而 slam_toolbox / Cartographer 属于图优化流派。两者选型的工程建议是：里程计质量差但计算资源少选粒子滤波，需要大规模建图和回环修正选图优化。

定位方面，Nav2 文档的 AMCL 配置页（configuring-amcl）是本例最佳参考：nav2_amcl 采用 KLD 自适应采样动态调整粒子数，默认推荐 `likelihood_field` 传感器模型，`alpha1-alpha4` 控制运动噪声，对应本节 10.6.1 的 AMCL 条目。

## 10.5 贝叶斯框架的数学推导

### 10.5.1 贝叶斯定理回顾

贝叶斯定理是SLAM的概率基础：

```
p(A|B) = p(B|A) · p(A) / p(B)
```

在SLAM上下文中：`p(x_t, m | z_{1:t}, u_{1:t-1})` 为后验概率（给定观测和控制的状态信念），`p(z_t | x_t, m)` 为似然（给定位姿和地图的观测概率），`p(x_t, m | z_{1:t-1}, u_{1:t-1})` 为先验（预测）。

### 10.5.2 递归贝叶斯滤波推导

SLAM使用递归贝叶斯滤波，通过预测-更新循环实现。**预测步骤** 如下：

```
bel(x_t, m) = ∫ p(x_t | x_{t-1}, u_t) · bel(x_{t-1}, m) dx_{t-1}
```

**更新步骤** 如下：

```
bel(x_t, m) = η · p(z_t | x_t, m) · bel(x_t, m)
```

### 10.5.3 独立性假设与简化

在FastSLAM中，利用条件独立性做出关键简化：

```
p(x_{1:t}, m | z_{1:t}, u_{1:t-1}) =
p(m | x_{1:t}, z_{1:t}) · p(x_{1:t} | z_{1:t}, u_{1:t-1})
```

这意味着：给定机器人轨迹，地图点条件独立，因此可先估计轨迹，再计算地图。

### 10.5.4 信息形式 vs 协方差形式

SLAM中两种等价的概率表示。**协方差形式（EKF-SLAM）** 维护状态均值和协方差矩阵，更新复杂度 O(n²)，直观表示不确定性；**信息形式（GraphSLAM）** 维护信息矩阵 (Ω) 和信息向量 (ξ)，稀疏信息矩阵利于大规模求解，使用迭代线性求解器。

以下类/函数片段摘自 [`bayes.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/bayes.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

原例运动预测为 pass，本例仍只演示线性观测的信息更新。对 z=Hx，信息向量应累加 HᵀR⁻¹z，不能先修改信息矩阵后再用创新量更新 xi。这里使用 Eigen 稠密矩阵便于教学，不作为大规模稀疏 SLAM 实现。

```cpp
class InformationFormSLAM {
public:
  MatrixXd Omega;
  VectorXd xi;
  explicit InformationFormSLAM(int num_landmarks = 100)
  : Omega(MatrixXd::Identity(3 + 2 * num_landmarks, 3 + 2 * num_landmarks) * 0.01),
    xi(VectorXd::Zero(3 + 2 * num_landmarks)) {}
  // 原例 predict 是 pass；本例只演示测量的信息贡献，不提供运动预测。
  void update_with_measurement(const Vector2d & z, int id, const Eigen::Matrix2d & R) {
    const int idx = 3 + 2 * id;
    MatrixXd H = MatrixXd::Zero(2, xi.size());
    H(0, 0) = -1.0; H(0, idx) = 1.0; H(1, 1) = -1.0; H(1, idx + 1) = 1.0;
    // 线性观测 z=H*x 的信息更新；信息向量累加 H^T R^-1 z。
    Omega += H.transpose() * R.inverse() * H;
    xi += H.transpose() * R.inverse() * z;
  }
  VectorXd get_mean() const { return Omega.ldlt().solve(xi); }
  MatrixXd get_covariance() const { return Omega.inverse(); }
};
```

## 10.6 ROS2 SLAM工具链概览

### 10.6.1 ROS2 SLAM生态

ROS2提供了完整的SLAM工具链。**slam_toolbox** 是ROS2官方2D SLAM解决方案，基于图优化，支持在线/离线建图，替代ROS1中的gmapping和karto；**nav2_amcl** 是基于已知地图的粒子滤波定位，采用KLD自适应采样，支持多种传感器模型；**cartographer_ros** 是Google开源的多传感器SLAM，支持2D和3D建图，激光+IMU融合。

### 10.6.2 常见启动命令

以下为三个可选系统，在 x86 课程容器执行；每次仅运行一个，先按章末实例启动仿真。

```bash
# x86课程容器已加载 /workspace/install/setup.bash
# slam_toolbox 在线异步建图
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$(ros2 pkg prefix slam_lab)/share/slam_lab/config/mapper_params_online_async.yaml" \
  use_sim_time:=true

# 课程 Cartographer 2D 建图
ros2 launch slam_lab cartographer_mapping.launch.py \
  configuration_directory:="$(ros2 pkg prefix slam_lab)/share/slam_lab/config/cartographer" \
  configuration_basename:=burger_2d.lua \
  use_sim_time:=true

# Nav2 AMCL定位
ros2 launch slam_lab amcl_localization.launch.py \
  map:=/workspace/output/maps/my_map.yaml use_sim_time:=true
```

### 10.6.3 数据录制与离线处理

在 x86 课程容器录制，约30秒后 Ctrl+C；停止实时仿真和建图后，两个独立容器终端分别运行同步建图与回放。回放完成后使用另一个终端保存地图，不能把以下常驻命令一次粘贴为串行脚本。

```bash
# x86课程容器
source /workspace/install/setup.bash
# 录制传感器数据
ros2 bag record --use-sim-time -o slam_data /scan /tf /tf_static /odom /imu

# 离线建图（使用rosbag）
# x86 容器终端 1；先停止实时 Gazebo 和原建图节点
ros2 launch slam_toolbox online_sync_launch.py \
  slam_params_file:="$(ros2 pkg prefix slam_lab)/share/slam_lab/config/mapper_params_online_async.yaml" \
  use_sim_time:=true

# 另一终端播放同一轮录制数据；回放时不同时接入实时 /clock
ros2 bag play slam_data --clock --rate 0.5

# 评估建图结果
ros2 run nav2_map_server map_saver_cli -f final_map
```

### 10.6.4 官方要点——官方 SLAM 工具链与 slam_toolbox 在线建图

docs.ros.org 官方 slam_toolbox 包文档与 SLAM Toolbox Wiki 是 ROS 2 上 2D SLAM 的事实标准参考，对应本节 10.6.1 的工具链概览。Wiki 明确把建图流程分为三种模式：`online_async`（在线异步建图，建图与定位可同时进行，适合边建图边使用的部署场景）、`online_sync`（同步处理扫描，机器人无需为每帧暂停）与 `offline`（在录制好的 rosbag 上一次性回放建图）。本节 10.6.2 的 `online_async_launch.py` 正是第一种模式，其关键参数 `use_sim_time:=true` 在 Gazebo 仿真中必须打开，否则传感器时间戳与仿真时钟不一致，扫描匹配会反复失败。

与本节 10.6.3 对应，slam_toolbox 支持把建好的位姿图（pose graph）序列化保存到磁盘，之后可重新加载续建——这是与 ROS 1 时代 gmapping「建完即弃」最大的工程差异。离线模式配合 rosbag 重放，是先录数据、后调参的标准工作流。

### 10.6.5 官方要点——参数调试与常见失败模式

Cartographer 官方教程给出了 2D 建图最重要的调参项：`trajectory_builder_2d` 的 `min_range`/`max_range`（裁剪激光有效测距范围，避免近处杂点与远处噪声）、`missing_data_ray_length`（无回波的射线长度）、`use_imu_data`（无 IMU 时必须关闭，否则位姿漂移）、相关性扫描匹配器的线性与角度搜索窗口，以及回环检测的 `spacing` 与后端优化频率。slam_toolbox 侧则需注意：对称走廊会引发「走廊误匹配」、里程计漂移过大会导致子图错位、地图发布周期过小会拖慢 RViz。

国际课程与官方文档共同强调两条工程原则：先在 rosbag 上离线调好参数，再上真机或仿真在线运行；回环被拒绝（loop closure rejected）时优先查激光范围裁剪与搜索窗口，而不是强行加大优化迭代次数。建议读者用本章课后练习第 4 题的离线建图流程，亲手对比不同 `resolution` 与 `max_laser_range` 下的建图效果。

## 课后练习

1. **原理题:** 简要说明SLAM问题的核心挑战，为什么定位和建图需要同时进行？用数学公式给出SLAM的后验概率表达。

2. **推导题:** 推导贝叶斯滤波的预测步骤和更新步骤公式，解释每个符号的物理含义。

3. **编程题:** 编写一个C++17函数，实现二维空间中基于贝叶斯滤波的位姿估计，输入为里程计数据和激光观测数据，输出为位姿后验均值。

4. **配置题:** 在一个已知环境的rosbag上，使用slam_toolbox进行离线建图，写出完整的命令行流程。

5. **分析题:** 比较基于滤波的SLAM方法和基于优化的SLAM方法的优缺点，说明各自适用场景。

6. **设计题:** 某巡检机器人需要在2000m²的仓库环境中运行，配备2D激光雷达、轮式里程计和IMU。为其设计SLAM方案，包括算法选择、传感器融合策略和参数配置建议。

---

## 仿真结合实例（当前仓库）：用 LiDAR、里程计和 TF 观察 SLAM 输入

### 目标与知识点对应

SLAM 的状态估计依赖观测 `z`、控制输入 `u` 和位姿状态 `x`。本实例不把仿真输出直接称为 SLAM 结果，而是使用 `sim_demo` 提供的真实 ROS 2 输入，检查贝叶斯滤波公式所需的数据流、时间戳和坐标关系。

### 运行步骤

仿真启动在 **x86 课程容器终端**执行，其他观测命令在 **COM260** 执行。x86 宿主仓库根目录先运行（每轮使用不同 RUN_ID）：

```bash
RUN_ID=teaching-slam
bash course_support/k3_com260_kit/scripts/x86-ch09-ch10.bash start "$RUN_ID"
bash course_support/k3_com260_kit/scripts/x86-ch09-ch10.bash exec "$RUN_ID" bash
```

容器终端已经加载 `/workspace/install/setup.bash`。按代码块标明的主机分别启动、观测，完成后先 Ctrl+C 停止节点，再在 x86 宿主执行 `bash course_support/k3_com260_kit/scripts/x86-ch09-ch10.bash stop "$RUN_ID"`。每次只运行一个建图/定位系统，防止多个节点同时发布 `map→odom`。


```bash
# x86 课程容器终端 1：启动仿真
source /workspace/install/setup.bash
ros2 launch sim_demo sim_bringup.launch.py \
  world:="$(ros2 pkg prefix sim_demo)/share/sim_demo/worlds/museum.sdf" \
  sensor_profile:=teacher gui:=true rviz:=true
```

```bash
# 终端 2：采样观测 z、控制/运动结果和 TF
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /tf --once
ros2 topic info /scan
```

在独立 COM260 终端启动自动运动演示；先观察小车周边空间，按 Ctrl+C 停止：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run param_demo_cpp patrol_driver --ros-args -p linear_speed:=0.12 -p angular_speed:=0.45
```

此终端接替原 `drive:=true` 的自动运动：复用第6章 C++ `patrol_driver`，线速度0.12 m/s、角速度0.45 rad/s，替代原仿真入口默认5.0 m/s、1.5 rad/s。传感器/TF观察目标不变，实际速度与轨迹以里程计为准；不要同时运行键盘控制。

### 观察结果

`/scan` 是激光观测，`/odom` 是里程计运动信息，`/tf` 提供坐标变换，三者共同构成 SLAM 节点的输入基础；比较消息的 `header.stamp` 和 `frame_id`，可以理解时间同步与坐标变换对状态估计的影响。

### 源码与边界

仿真与桥接入口为 `src_k3_com260_kit/sim_demo/launch/sim_bringup.launch.py` 和 `src_k3_com260_kit/sim_demo/config/bridge.yaml`，SLAM 运行入口为 `src_k3_com260_kit/slam_lab/launch/online_mapping.launch.py`。本节只验证 SLAM 输入链路；本章数值步骤可用`ros2 run slam_teaching_cpp bayes_demo`验证，完整在线SLAM使用后续工具链入口。

---

学习材料：
- ROS 2 官方文档 —— slam_toolbox 包文档：https://docs.ros.org/
- SLAM Toolbox Wiki —— Steve Macenski 维护：https://github.com/SteveMacenski/slam_toolbox/wiki
- Google Cartographer 官方文档：https://google-cartographer-ros.readthedocs.io/
- Nav2 官方文档 —— AMCL 配置：https://docs.nav2.org/
- The Construct —— ROS 2 与 SLAM 相关课程：https://www.theconstructsim.com/
