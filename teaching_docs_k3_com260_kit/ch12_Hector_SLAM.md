# 第12章 Hector SLAM

本章为 COM260 Kit 的 C++17 教学副本。课程概念、练习及数值要求沿用原章；实测状态与素材见[实验第10章](../lab_manuals_k3_com260_kit/ch10_lab.md)和[运行证据](../lab_manuals_k3_com260_kit/runtime_evidence.md#ch10)。算法片段展示相应数学步骤，不能代替完整 SLAM 系统。

数值示例和 ROS 已知输入已分别在 COM260 与 x86 构建运行。原始记录：[COM260数值](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-numerics.cast)、[COM260 ROS接口](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-ros.cast)、[真实仿真传感器](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-live-sensors.cast)。这些检查的输入和范围分开记录，不能据此宣称所有教学算法均在真实场景达到定位精度要求。

教学程序位于 [`slam_teaching_cpp`](../src_k3_com260_kit/slam_teaching_cpp/README.md)。COM260 终端先执行：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
```


> **课程**：ROS2 C++17 编程
> **章节**：第12章
> **课时**：2 课时（90 分钟）
> **教学方式**：讲授 + 演示

---

## 学习目标

本章学习目标包括：理解Hector-SLAM的基本原理和算法架构；掌握基于优化的激光SLAM方法；理解双栅格地图和多分辨率匹配策略；熟悉Gauss-Newton地图匹配的数学推导；能够分析Hector-SLAM的优缺点和适用场景。

## 12.1 Hector-SLAM算法概述

### 12.1.1 算法背景

Hector-SLAM是一种基于优化的2D SLAM方法，由德国达姆施塔特理工大学提出。与传统的基于滤波的SLAM方法不同，Hector-SLAM采用扫描-地图匹配（Scan-to-Map）策略，不需要里程计信息。

**核心思想：** 最小化当前激光扫描与已有栅格地图之间的灰度差异，通过Gauss-Newton迭代优化求解机器人位姿。

**主要特点：** Hector-SLAM不依赖里程计，可在手持设备或无轮式编码器的机器人上运行；使用多分辨率栅格地图加速匹配；基于双线性插值的连续地图表示。

### 12.1.2 Hector-SLAM的架构

Hector-SLAM的系统架构分为三个主要模块：

```
激光扫描数据输入
    ↓
┌─────────────────────────┐
│    扫描-地图匹配        │
│  (Gauss-Newton优化)     │
│  输入: 当前扫描+已有地图 │
│  输出: 优化后位姿       │
└─────────┬───────────────┘
          ↓
┌─────────────────────────┐
│    地图更新             │
│  (双栅格分辨率)          │
│  输入: 优化后的位姿+扫描 │
│  输出: 更新后的栅格地图  │
└─────────┬───────────────┘
          ↓
┌─────────────────────────┐
│    地图发布             │
│  /map (OccupancyGrid)   │
│  /slam_out_pose (Pose)  │
└─────────────────────────┘
```

### 12.1.3 Hector-SLAM的优缺点

**优点：** 无需里程计，适用性广泛；计算效率高，可实时运行；在结构化环境中精度高；实现相对简单。

**缺点：** 依赖高精度激光雷达，低速率的激光会导致漂移；速度过快会导致建图漂移；回环检测能力弱；对初值敏感，重定位能力差。

| 对比维度 | Hector-SLAM | gmapping | Cartographer |
|---------|-------------|----------|-------------|
| 里程计依赖 | 不需要 | 需要 | 可选 |
| 回环检测 | 不支持 | 不支持 | 支持 |
| 建图质量 | 依赖激光频率 | 依赖里程计精度 | 高（图优化） |
| 计算效率 | 高 | 中 | 中 |
| 适用场景 | 手持/无里程计 | 有里程计 | 复杂大场景 |

## 12.2 扫描-地图匹配算法

### 12.2.1 目标函数定义

Hector-SLAM的扫描-地图匹配通过最小化以下误差函数来优化位姿：

```
T* = argmin Σ [1 - M(S_i(T))]²
```

其中，T = (x, y, θ) 为待优化的机器人位姿；S_i(T) 为第i个激光点在变换T下的地图坐标；M(S_i(T)) 为地图在坐标S_i(T)处的占据概率值（0=空闲，1=占据）。

**目标函数的直观理解：** 当激光点落在占据区域时M≈1、误差≈0；当激光点落在空闲区域时M≈0、误差≈1；优化目标是所有激光点都匹配到地图的占据区域。

### 12.2.2 Gauss-Newton优化求解

由于目标函数是非线性的，使用Gauss-Newton法迭代求解：

```
ΔT = H⁻¹ · ∇M · Σ (∂S_i/∂T)ᵀ · [1 - M(S_i(T))]
```

其中，∇M 为地图M在S_i处的梯度；∂S_i/∂T 为激光点对位姿参数的雅可比；H 为Hessian矩阵近似：H = Σ J_iᵀ · J_i。

以下类/函数片段摘自 [`hector.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/hector.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

地图梯度及15次/1e−6收敛条件保持原例。新增显式地图原点参数：栅格坐标为(world−origin)/resolution；默认原点仍为(0,0)。这修正了后文居中地图与本匹配器坐标不一致的问题。

```cpp
class HectorScanMatcher {
public:
  int max_iterations; double convergence_delta;
  explicit HectorScanMatcher(int iterations = 15, double delta = 1e-6)
  : max_iterations(iterations), convergence_delta(delta) {}
  static double interpolate(const MatrixXd & map, const Vector2d & p, double outside = 0.5) {
    int x = static_cast<int>(std::floor(p.x())), y = static_cast<int>(std::floor(p.y()));
    if (x < 0 || y < 0 || x + 1 >= map.cols() || y + 1 >= map.rows()) { return outside; }
    double dx = p.x() - x, dy = p.y() - y;
    return map(y, x) * (1-dx) * (1-dy) + map(y, x+1) * dx * (1-dy) +
      map(y+1, x) * (1-dx) * dy + map(y+1, x+1) * dx * dy;
  }
  Vector3d match(const Points & scan, const MatrixXd & map, double resolution,
    Vector3d pose, const Vector2d & origin = Vector2d::Zero()) const {
    if (resolution <= 0 || map.rows() < 2 || map.cols() < 2) {
      throw std::invalid_argument("地图分辨率必须为正，地图至少为2×2");
    }
    MatrixXd gx(map.rows(), map.cols()), gy(map.rows(), map.cols());
    for (int y = 0; y < map.rows(); ++y) { for (int x = 0; x < map.cols(); ++x) {
      const int x0 = std::max(0, x-1), x1 = std::min<int>(map.cols()-1, x+1);
      const int y0 = std::max(0, y-1), y1 = std::min<int>(map.rows()-1, y+1);
      gx(y, x) = (map(y, x1)-map(y, x0)) / ((x1-x0) * resolution);
      gy(y, x) = (map(y1, x)-map(y0, x)) / ((y1-y0) * resolution);
    }}
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
      const Eigen::Matrix2d R = rotation(pose.z());
      Eigen::Matrix2d dR; dR << -std::sin(pose.z()), -std::cos(pose.z()), std::cos(pose.z()), -std::sin(pose.z());
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero(); Vector3d b = Vector3d::Zero();
      for (const auto & point : scan) {
        Vector2d grid = (R * point + pose.head<2>() - origin) / resolution;
        const double residual = 1.0 - interpolate(map, grid);
        Vector2d gradient(interpolate(gx, grid, 0.0), interpolate(gy, grid, 0.0));
        Vector3d J(-gradient.x(), -gradient.y(), -gradient.dot(dR * point));
        H += J * J.transpose(); b += J * residual;
      }
      Vector3d delta = (H + 1e-6 * Eigen::Matrix3d::Identity()).ldlt().solve(-b);
      pose += delta;
      if (delta.norm() < convergence_delta) { break; }
    }
    return pose;
  }
};
```

### 12.2.3 双线性地图插值

Hector-SLAM的一个关键创新是使用**双线性插值**将离散栅格地图转换为连续可微函数：

```
M(P) = Σ w_ij · M_ij
```

其中 w_ij 为双线性插值权重，使得地图函数可导，支持Gauss-Newton优化。

**双线性插值图示：**
```
M00 ─── dx ─── M10
  │             │
 dy    M(P)     dy
  │             │
M01 ─── dx ─── M11

M(P) = M00·(1-dx)·(1-dy) + M10·dx·(1-dy) + M01·(1-dx)·dy + M11·dx·dy
```

### 12.2.4 扫描匹配性能分析

以下类/函数片段摘自 [`analysis_examples.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/analysis_examples.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
inline void analyze_scan_matching(const MatrixXd & map,const Points & scan) {
  const std::vector<Vector3d> offsets{Vector3d(0,0,0),Vector3d(0.1,0.1,0.05),Vector3d(0.5,0.5,0.2),Vector3d(1,1,0.5)};
  for (const auto & offset:offsets) {
    const auto optimized=HectorScanMatcher().match(scan,map,0.05,Vector3d(5,5,0)+offset);
    const double error=(optimized-Vector3d(5,5,0)).norm();
    std::cout << "offset=" << offset.transpose() << " final_error=" << error << " converged=" << (error<0.1) << '\n';
  }
}
```

## 12.3 双栅格地图与多分辨率匹配

### 12.3.1 多分辨率地图策略

Hector-SLAM使用两级（或多级）分辨率栅格地图来提高匹配效率和鲁棒性：

```
粗分辨率地图 (0.2m/像素):
┌──────────────────────┐
│  快速粗匹配           │
│  获取初始位姿估计      │
│  收敛域大，精度低     │
└──────────────────────┘
         ↓ (结果作为初值)
┌──────────────────────┐
│  高分辨率地图 (0.05m/像素) │
│  精确匹配             │
│  收敛域小，精度高     │
└──────────────────────┘
         ↓
    最终精确位姿
```

以下类/函数片段摘自 [`hector.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/hector.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
class MultiResolutionMap {
public:
  std::vector<double> resolutions{0.20, 0.10, 0.05};
  std::map<double, MatrixXd> maps;
  void build_pyramid(const MatrixXd & base_map) {
    for (double resolution : resolutions) {
      const int scale = static_cast<int>(std::round(resolution / resolutions.back()));
      MatrixXd downsampled(base_map.rows() / scale, base_map.cols() / scale);
      for (int y = 0; y < downsampled.rows(); ++y) { for (int x = 0; x < downsampled.cols(); ++x) {
        downsampled(y, x) = base_map.block(y * scale, x * scale, scale, scale).mean();
      }}
      maps[resolution] = downsampled;
    }
  }
  const MatrixXd & get_map(double resolution) const {
    auto closest = std::min_element(resolutions.begin(), resolutions.end(),
      [resolution](double a, double b) { return std::abs(a-resolution) < std::abs(b-resolution); });
    return maps.at(*closest);
  }
};
```

### 12.3.2 由粗到精的匹配策略

以下类/函数片段摘自 [`hector.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/hector.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
class CoarseToFineMatcher {
public:
  Vector3d match(const Points & scan, const MultiResolutionMap & pyramid, Vector3d pose,
    const Vector2d & origin = Vector2d::Zero()) const {
    for (double resolution : pyramid.resolutions) {
      pose = HectorScanMatcher().match(scan, pyramid.maps.at(resolution), resolution, pose, origin);
      std::cout << "分辨率 " << resolution << "m: 位姿=" << pose.transpose() << '\n';
    }
    return pose;
  }
};
```

### 12.3.3 地图更新策略

Hector-SLAM的地图更新是一个增量过程，每当获得新位姿后，就将激光扫描数据融合到地图中：

以下类/函数片段摘自 [`hector.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/hector.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

保留20 m示例尺寸、0.05 m分辨率、空闲−0.3/占据+1.0、log-odds截断±5、占据阈值0.2/0.8。边界栅格采用floor，避免地图外负坐标被截断到0。传入点须已变换到机器人坐标系；原例未使用的laser_pose不作为生效参数。

```cpp
class OccupancyGridMap {
public:
  double resolution; int width_pixels, height_pixels; Vector2d origin;
  MatrixXd log_odds;
  OccupancyGridMap(double width, double height, double res)
  : resolution(res), width_pixels(static_cast<int>(width/res)), height_pixels(static_cast<int>(height/res)),
    origin(-width/2, -height/2), log_odds(MatrixXd::Zero(height_pixels, width_pixels)) {}
  Eigen::Vector2i world_to_grid(const Vector2d & point) const {
    return ((point - origin) / resolution).array().floor().cast<int>().matrix();
  }
  Vector2d grid_to_world(const Eigen::Vector2i & cell) const { return cell.cast<double>() * resolution + origin; }
  bool inside(const Eigen::Vector2i & cell) const {
    return cell.x() >= 0 && cell.y() >= 0 && cell.x() < width_pixels && cell.y() < height_pixels;
  }
  static std::vector<Eigen::Vector2i> bresenham_line(Eigen::Vector2i from, const Eigen::Vector2i & to) {
    std::vector<Eigen::Vector2i> cells;
    const int dx = std::abs(to.x()-from.x()), dy = -std::abs(to.y()-from.y());
    const int sx = from.x() < to.x() ? 1 : -1, sy = from.y() < to.y() ? 1 : -1;
    int error = dx + dy;
    while (true) {
      cells.push_back(from); if (from == to) { break; }
      int e2 = 2 * error;
      if (e2 >= dy) { error += dy; from.x() += sx; }
      if (e2 <= dx) { error += dx; from.y() += sy; }
    }
    return cells;
  }
  void update_with_scan(const Points & scan, const Vector3d & pose) {
    for (const auto & point : scan) {
      Eigen::Vector2i end = world_to_grid(rotation(pose.z()) * point + pose.head<2>());
      if (!inside(end)) { continue; }
      auto cells = bresenham_line(world_to_grid(pose.head<2>()), end);
      for (std::size_t i = 0; i + 1 < cells.size(); ++i) {
        if (inside(cells[i])) { log_odds(cells[i].y(), cells[i].x()) -= 0.3; }
      }
      log_odds(end.y(), end.x()) += 1.0;
    }
    log_odds = log_odds.array().max(-5.0).min(5.0).matrix();
  }
  MatrixXd get_probability_map() const { return (1.0 / (1.0 + (-log_odds.array()).exp())).matrix(); }
  std::vector<int8_t> get_occupancy_grid() const {
    const auto probability = get_probability_map(); std::vector<int8_t> data;
    for (int y = 0; y < height_pixels; ++y) { for (int x = 0; x < width_pixels; ++x) {
      double p = probability(y, x); data.push_back(p < 0.2 ? 0 : p > 0.8 ? 100 : -1);
    }}
    return data;
  }
};
```

### 12.3.4 官方要点——多分辨率匹配与双线性插值的工程价值

原论文（Kohlbrecher 等，IROS 2011，官方仓库 README 引用）给出了两个被后续系统广泛借鉴的设计：用双线性插值把占据栅格变成连续可微的地图函数（本章 12.2.3），以及用多分辨率地图做由粗到精匹配（本章 12.3）。第二点与 Cartographer 文档中的相关性粗匹配异曲同工：先在大收敛域上把初值拉进正确「盆地」，再在细分辨率上精化——理解了 12.3.1 的金字塔策略，就同时理解了工业级 SLAM 里「粗配准+精配准」的通用套路。

## 12.4 Hector-SLAM的ROS2实现

### 12.4.1 ROS2节点设计

在ROS2中，Hector-SLAM可以作为独立节点运行：

完整程序：[`hector_slam.cpp`](../src_k3_com260_kit/slam_teaching_cpp/src/hector_slam.cpp)。

这是本章教学匹配器的ROS封装，不是官方hector_mapping。先将激光点按TF变换到base_link；匹配使用地图原点；map→odom按map→base_link×inverse(odom→base_link)计算。原例直接把map→base位姿发布成map→odom会重复叠加里程计，已修正。update_factor_free/occupied仍按原声明保留，但原教学地图使用固定log-odds增量，这两个参数不参与更新。勿与其他/map及map→odom发布者同时运行。

```cpp
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "slam_teaching_cpp/hector.hpp"

class HectorSLAMNode : public rclcpp::Node {
public:
  HectorSLAMNode() : Node("hector_slam"), buffer_(get_clock()), listener_(buffer_), broadcaster_(*this) {
    const auto resolution = declare_parameter("map_resolution", 0.05);
    const auto size = declare_parameter("map_size", 20.0);
    matcher_.max_iterations = declare_parameter("max_iterations", 15);
    // 沿用原示例参数声明；本教学地图更新固定使用 log-odds -0.3/+1.0。
    declare_parameter("update_factor_free", 0.4); declare_parameter("update_factor_occupied", 0.9);
    if (resolution <= 0 || size <= 2*resolution || matcher_.max_iterations <= 0) {
      throw std::invalid_argument("map_resolution/map_size/max_iterations 无效");
    }
    map_ = std::make_unique<slam_teaching::OccupancyGridMap>(size, size, resolution);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/slam_out_pose", 10);
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(*msg); });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish_map(); });
    RCLCPP_INFO(get_logger(), "Hector-SLAM教学节点已启动");
  }
private:
  void on_scan(const sensor_msgs::msg::LaserScan & msg) {
    try {
      const auto laser = buffer_.lookupTransform("base_link", msg.header.frame_id, tf2::TimePointZero);
      tf2::Transform base_from_laser; tf2::fromMsg(laser.transform, base_from_laser);
      slam_teaching::Points points;
      for (std::size_t i = 0; i < msg.ranges.size(); ++i) {
        const double r = msg.ranges[i], angle = msg.angle_min + i * msg.angle_increment;
        if (std::isfinite(r) && r > msg.range_min && r < msg.range_max) {
          auto p = base_from_laser * tf2::Vector3(r*std::cos(angle), r*std::sin(angle), 0);
          points.emplace_back(p.x(), p.y());
        }
      }
      if (points.size() < 10) { return; }
      if (first_scan_) {
        map_->update_with_scan(points, pose_); first_scan_ = false;
        RCLCPP_INFO(get_logger(), "首帧扫描已处理"); return;
      }
      pose_ = matcher_.match(points, map_->get_probability_map(), map_->resolution, pose_, map_->origin);
      map_->update_with_scan(points, pose_);
      geometry_msgs::msg::PoseStamped output;
      output.header.frame_id = "map"; output.header.stamp = msg.header.stamp;
      output.pose.position.x = pose_.x(); output.pose.position.y = pose_.y();
      output.pose.orientation.z = std::sin(pose_.z()/2); output.pose.orientation.w = std::cos(pose_.z()/2);
      pose_pub_->publish(output);
      // map→odom = map→base_link × inverse(odom→base_link)。
      const auto odom = buffer_.lookupTransform("odom", "base_link", rclcpp::Time(msg.header.stamp));
      tf2::Transform odom_from_base; tf2::fromMsg(odom.transform, odom_from_base);
      tf2::Transform map_from_base; tf2::fromMsg(output.pose, map_from_base);
      geometry_msgs::msg::TransformStamped tf;
      tf.header = output.header; tf.child_frame_id = "odom";
      tf.transform = tf2::toMsg(map_from_base * odom_from_base.inverse()); broadcaster_.sendTransform(tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF等待: %s", e.what());
    }
  }
  void publish_map() {
    nav_msgs::msg::OccupancyGrid msg; msg.header.frame_id = "map"; msg.header.stamp = now();
    msg.info.resolution = map_->resolution; msg.info.width = map_->width_pixels; msg.info.height = map_->height_pixels;
    msg.info.origin.position.x = map_->origin.x(); msg.info.origin.position.y = map_->origin.y();
    msg.info.origin.orientation.w = 1.0; msg.data = map_->get_occupancy_grid(); map_pub_->publish(msg);
  }
  slam_teaching::Vector3d pose_ = slam_teaching::Vector3d::Zero(); bool first_scan_ = true;
  std::unique_ptr<slam_teaching::OccupancyGridMap> map_; slam_teaching::HectorScanMatcher matcher_;
  tf2_ros::Buffer buffer_; tf2_ros::TransformListener listener_; tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<HectorSLAMNode>()); rclcpp::shutdown();
}
```

### 12.4.2 启动配置

```bash
# COM260：本章运行独立C++教学实现；官方hector_mapping的ROS1参数仅作对照
source ~/.config/ros2-course-com260-ch09-ch10/env.bash

# 启动Hector-SLAM
ros2 launch slam_teaching_cpp hector_teaching.launch.py

# 或者在自定义launch文件中配置
```

完整 Launch：[`hector_teaching.launch.py`](../src_k3_com260_kit/slam_teaching_cpp/launch/hector_teaching.launch.py)。

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_teaching_cpp',
            executable='hector_slam',
            name='hector_slam',
            output='screen',
            parameters=[{
                'map_resolution': 0.05,
                'map_size': 20.0,
                'max_iterations': 15,
                'update_factor_free': 0.4,
                'update_factor_occupied': 0.9,
                'use_sim_time': True,
            }],
        ),
    ])
```

### 12.4.3 参数调优指南

Hector-SLAM的关键参数及其调优建议：

| 参数 | 默认值 | 说明 | 调优建议 |
|------|--------|------|---------|
| map_resolution | 0.05 | 地图分辨率(m) | 大场景增大到0.1，精细场景减小到0.025 |
| map_size | 20.0 | 地图尺寸(m) | 根据环境大小调整 |
| max_iterations | 15 | 优化最大迭代次数 | 激光频率高可减少 |
| update_factor_free | 0.4 | 原接口对照参数；本教学实现未使用 | 本例固定log-odds −0.3 |
| update_factor_occupied | 0.9 | 原接口对照参数；本教学实现未使用 | 本例固定log-odds +1.0 |

### 12.4.4 官方要点——hector_slam 官方包：节点构成与 frame 约定

hector_slam 由德国达姆施塔特理工大学（TU Darmstadt）团队以 `tu-darmstadt-ros-pkg` 生态发布，其 ROS 1 Wiki 页面至今仍是事实上的官方教程。核心包 `hector_mapping` 提供本章 12.4 所示的节点接口：订阅 `scan_topic`（默认 `/scan`），发布 `/map` 与 `/slam_out_pose`。frame 约定与本章 12.4.2 启动参数一一对应：`base_frame`（如 `base_footprint`）、`odom_frame` 与 `map_frame` 三个参数决定 TF 树的挂接方式；`pub_map_odom_transform` 打开时由建图节点直接广播 map→odom（本章 `broadcast_tf` 的行为），关闭时则需要外部定位节点补齐该变换。

在 ROS 2 发行版中，hector_slam 长期停留在社区移植或源码构建阶段，官方 Wiki 也没有 ROS 2 专属教程。因此将官方hector_mapping参数与本章独立教学实现区分的做法，正是遵循官方文档对可用性的提示；若不可用，社区普遍建议改用 slam_toolbox 或 Cartographer，它们覆盖同样的扫描-地图匹配思想且维护活跃。

### 12.4.5 官方要点——参数调优与建图质量评估

官方 Wiki 的 hector_mapping 参数表对应本章 12.4.3：`map_resolution`（0.025~0.1 m 之间权衡内存与精度）、`map_size`（按环境外扩）、`update_factor_free/occupied`（对数几率增减幅度，本章 12.3.3 中 0.3/1.0 的演示值即其来源）、`max_iterations`（高帧率雷达可适当减小以控延迟）。调参纪律与第 10 章扩展一致：先用 `ros2 bag record` 录制 `/scan` 与 `/tf`，离线重放试参，每次只改一个变量。

质量评估可复用 ROS 2 官方工具链：建图完成后用 `nav2_map_server` 的 `map_saver_cli` 保存（本章 12.5.1），再与已知平面图对比占据边界偏移；对漂移的直观检查是把 `map→odom` TF 在 RViz 中长期观察其缓慢累积的旋转量。建议读者按本章练习第 5 题在仿真中系统复现这些影响。

## 12.5 仿真与实践

### 12.5.1 Gazebo仿真建图

先按本章末尾仿真实例准备 x86 课程容器。每条常驻命令使用独立终端：

**1. x86课程容器：启动仿真环境**

```bash
source /workspace/install/setup.bash
ros2 launch sim_demo sim_bringup.launch.py \
  world:="$(ros2 pkg prefix sim_demo)/share/sim_demo/worlds/museum.sdf" \
  sensor_profile:=teacher gui:=true rviz:=true
```

**2. COM260：启动建图**

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 launch slam_teaching_cpp hector_teaching.launch.py
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
ros2 run nav2_map_server map_saver_cli -f /workspace/output/maps/hector_map --ros-args -p use_sim_time:=true
```

检查生成的 `.pgm` 与 `.yaml`，对照当次 RViz 地图。先停止运动，再停止建图与仿真。

### 12.5.2 在已知地图上定位

完整程序：[`hector_localization.cpp`](../src_k3_com260_kit/slam_teaching_cpp/src/hector_localization.cpp)。

补齐原例未初始化的current_pose和缺失的LaserScan类型，并使用已知地图原点。这里估计激光坐标系位姿，初始猜测为零；仅演示已知图扫描匹配，不发布TF，也不替代完整定位系统。

```cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "slam_teaching_cpp/hector.hpp"
class HectorLocalization : public rclcpp::Node {
public:
  HectorLocalization() : Node("hector_localization") {
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        if (map_.size() != 0) { return; }
        resolution_ = msg->info.resolution;
        origin_ << msg->info.origin.position.x, msg->info.origin.position.y;
        map_.resize(msg->info.height, msg->info.width);
        for (int y = 0; y < map_.rows(); ++y) { for (int x = 0; x < map_.cols(); ++x) {
          const auto value = msg->data.at(y * msg->info.width + x); map_(y, x) = value < 0 ? 0.5 : value / 100.0;
        }}
        RCLCPP_INFO(get_logger(), "已知地图已加载: %ux%u", msg->info.width, msg->info.height);
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        if (map_.size() == 0) { return; }
        slam_teaching::Points points;
        for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
          const double r = msg->ranges[i], angle = msg->angle_min + i * msg->angle_increment;
          if (std::isfinite(r)) { points.emplace_back(r * std::cos(angle), r * std::sin(angle)); }
        }
        current_pose_ = slam_teaching::HectorScanMatcher().match(points, map_, resolution_, current_pose_, origin_);
        RCLCPP_INFO(get_logger(), "激光位姿=(%.3f,%.3f,%.3f)", current_pose_.x(), current_pose_.y(), current_pose_.z());
      });
  }
private:
  slam_teaching::MatrixXd map_; double resolution_ = 0;
  slam_teaching::Vector2d origin_ = slam_teaching::Vector2d::Zero();
  slam_teaching::Vector3d current_pose_ = slam_teaching::Vector3d::Zero();
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<HectorLocalization>()); rclcpp::shutdown();
}
```

### 12.5.3 常见问题与解决方案

**问题1：建图漂移**：机器人移动过快或激光帧率不足可能造成漂移，可通过降低移动速度、增加激光频率排查。但本章简化示例在静止时也观测到位姿漂移，不能据此认为降速必然解决；见[教学 Hector 实测结果](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-hector-result.json)。该记录不单独确定漂移原因。

**问题2：地图失真**：原因是初始位姿误差过大导致匹配失败，解决方法是确保初始位姿估计准确，或增加粗分辨率层。

**问题3：重定位失败**：本章教学 `hector_slam`、`hector_localization` 节点不订阅 `/initialpose`，不支持通过该话题复位。需要已知地图重定位时，先保存地图并退出 Hector 节点，停止它发布的 `/map` 和 `map→odom`，再按第 14 章启动 map_server/AMCL；避免两套定位同时发布同名 TF。以下命令只用于已激活、已加载地图的 AMCL：

```bash
# 向AMCL发送原点初值（x/y方差0.25m²，yaw方差约0.0685rad²）
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0685]}}"
```

### 12.5.4 官方要点——无里程计建图：适用与失效的边界

Hector SLAM 最大的卖点是不依赖里程计，官方 Wiki 明确给出其设计场景：手持建图设备、无编码器的平台（如部分无人机、扫地原型机）。但这也带来两个边界条件，与本章 12.5.3 的问题清单完全吻合：其一，初值完全靠「上一帧结果」外推，要求激光帧率高（官方建议 10 Hz 以上）且角速度受限，否则快速转身会直接导致建图漂移；其二，缺乏全局重定位能力，丢失后可重启教学节点从初始状态建图，或切换到 AMCL 进行已知地图定位；12.5.3 的 `/initialpose` 命令针对后者，不会复位本章 Hector 示例。

对比理解：slam_toolbox 的 Ceres 匹配器与 Hector 同属「扫描-地图 + Gauss-Newton 派」（见本书第 11 章扩展），但多了里程计先验与回环后端；Cartographer 则用实时相关性匹配（粗）+ 非线性优化（精）的双层结构显式扩大初值容错窗。三者可视为本章 12.1.3 对比表在实现层面的注脚。

## 课后练习

1. **原理题:** 说明Hector-SLAM为什么不需要里程计信息？它的扫描-地图匹配策略相比帧-帧匹配有什么优势？

2. **推导题:** 推导Hector-SLAM中Gauss-Newton优化的雅可比矩阵，包括旋转参数θ的导数和地图梯度∇M的计算。

3. **编程题:** 实现一个简化版Hector-SLAM系统的扫描匹配模块，能够对模拟激光数据进行扫描-地图匹配并输出优化位姿。

4. **分析题:** 分析Hector-SLAM中双栅格地图的作用，说明由粗到精匹配策略如何提高匹配的鲁棒性和效率。

5. **配置题:** 在Gazebo仿真中启动Hector-SLAM，调整参数（地图分辨率、更新因子等），观察不同参数对建图质量的影响。

6. **设计题:** 某手持建图设备（无里程计）需要在2000m²的办公环境中快速建图。设计方案包括激光雷达选型、Hector-SLAM参数配置、建图路径规划策略和地图质量评估方法。

---

## 仿真结合实例（当前仓库）：为 Hector SLAM 准备二维激光输入

### 目标与知识点对应

Hector SLAM 依赖高频激光和 TF。本章提供独立 C++ 教学节点 `slam_teaching_cpp/hector_slam`；实例先使用 `sim_demo` 验证 `/scan`、`/tf` 和仿真时钟，再按12.5.1接入教学节点。已知输入检查覆盖消息、栅格和TF计算，不代表真实运动中的定位精度通过；重复静止扫描仍可能产生漂移。

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
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_link laser_link
```

在独立 COM260 终端启动自动运动演示；先观察小车周边空间，按 Ctrl+C 停止：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run param_demo_cpp patrol_driver --ros-args -p linear_speed:=0.12 -p angular_speed:=0.45
```

此终端接替原 `drive:=true` 的自动运动：复用第6章 C++ `patrol_driver`，线速度0.12 m/s、角速度0.45 rad/s，替代原仿真入口默认5.0 m/s、1.5 rad/s。传感器/TF观察目标不变，实际速度与轨迹以里程计为准；不要同时运行键盘控制。

### 观察结果

RViz 中可同时显示 LaserScan 和 TF；外部 Hector 节点应将激光 frame 与机器人基座 frame 配置一致，并使用 `use_sim_time`。

### 源码与边界

仿真模型与桥接配置位于 `src_k3_com260_kit/sim_demo/models/burger_teacher.sdf` 与 `src_k3_com260_kit/sim_demo/config/bridge.yaml`；可替代的在线建图示例见 `src_k3_com260_kit/slam_lab/`。

独立教学实现位于 `src_k3_com260_kit/slam_teaching_cpp/src/hector_slam.cpp`；其结果与官方 Hector、slam_toolbox 分别记录。

学习材料：
- ROS Wiki —— hector_slam（Tutorials 与 hector_mapping 参数说明）：https://wiki.ros.org/hector_slam
- GitHub（tu-darmstadt-ros-pkg）—— hector_slam 官方仓库与 README：https://github.com/tu-darmstadt-ros-pkg/hector_slam
- docs.ros.org —— nav2_map_server map_saver_cli 用法：https://docs.ros.org/en/humble/p/nav2_map_server/
- The Construct —— ROS 2 SLAM 相关课程（Hector 与 slam_toolbox 对比）：https://www.theconstructsim.com/
- Robotics Back-End —— 激光 SLAM 建图教程与实测对比：https://roboticsbackend.com/
