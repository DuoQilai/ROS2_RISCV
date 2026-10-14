# 第15章 Cartographer图优化SLAM

本章为 COM260 Kit 的 C++17 教学副本。课程概念、练习及数值要求沿用原章；实测状态与素材见[实验第10章](../lab_manuals_k3_com260_kit/ch10_lab.md)和[运行证据](../lab_manuals_k3_com260_kit/runtime_evidence.md#ch10)。算法片段展示相应数学步骤，不能代替完整 SLAM 系统。

数值示例和 ROS 已知输入已分别在 COM260 与 x86 构建运行。原始记录：[COM260数值](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-numerics.cast)、[COM260 ROS接口](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-ros.cast)、[真实仿真传感器](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-live-sensors.cast)。这些检查的输入和范围分开记录，不能据此宣称所有教学算法均在真实场景达到定位精度要求。

教学程序位于 [`slam_teaching_cpp`](../src_k3_com260_kit/slam_teaching_cpp/README.md)。COM260 终端先执行：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
```


> **课程**：ROS2 C++17 编程
> **章节**：第15章
> **课时**：6 课时（4 理论 + 2 实验）
> **教学方式**：讲授 + 演示

---

## 学习目标

本章学习目标包括：理解图优化SLAM的基本原理和数学框架，掌握Cartographer系统的整体架构和设计思想，熟悉局部SLAM（前端匹配）算法流程，理解全局SLAM（后端优化）和回环检测机制，能够在ROS2中配置和使用Cartographer进行多传感器融合建图，掌握Cartographer参数调优方法。

## 15.1 图优化SLAM基本原理

### 15.1.1 从滤波到图优化

传统滤波SLAM（EKF-SLAM, FastSLAM）存在两个根本性问题：**累积误差**，即只维护当前状态估计，过去的误差无法修正；**线性化误差**，即系统线性化只发生在当前估计点，长期误差大。

图优化SLAM（Graph-based SLAM）通过记录所有位姿和约束，构建全局优化问题来消除累积误差。

**核心思想：**将SLAM问题建模为图（Graph），图的节点（Vertex）为机器人位姿和路标点，图的边（Edge）为位姿间的约束（里程计、观测、回环），优化目标为最小化所有约束误差的平方和。

```
图结构示意:
   x1 ──── x2 ──── x3 ──── x4 ──── x5
    │       │       │       │       │
    │   ┌───┘       │       └───┐   │
    │   │           │           │   │
   lm1  lm2        lm3         lm4 lm5

x: 机器人位姿节点
lm: 路标点节点
─: 里程计边
│: 观测边
```

### 15.1.2 图优化的数学形式

**优化目标函数：**

```
X* = argmin Σ e_ij(X)ᵀ · Ω_ij · e_ij(X)
```

其中：X = {x₁, x₂, ..., xₙ, l₁, l₂, ..., lₘ} 为所有待优化变量，e_ij(X) 为约束ij的误差向量，Ω_ij 为约束ij的信息矩阵（协方差的逆）。

**对于位姿图（Pose Graph）：**
```
e_ij(x_i, x_j) = t_ij ⊖ (x_i ⊖ x_j)
```

其中 ⊖ 表示位姿复合运算的逆，t_ij 为观测到的相对变换。

### 15.1.3 稀疏性利用

图优化SLAM的关键优势在于**稀疏性**：

```
Hessian矩阵结构:
[x₁ x₂ x₃ ... xₙ l₁ l₂ ... lₘ]
[x₁  X  X  .  .  .  X  .  .  .]
[x₂  X  X  X  .  .  .  X  .  .]
[x₃  .  X  X  X  .  .  .  X  .]
[.   .  .  X  X  .  .  .  .  .]
[l₁  X  .  .  .  .  X  .  .  .]
[l₂  .  X  .  .  .  .  X  .  .]
```

由于每个约束只关联少量节点，H矩阵大部分为0，可使用稀疏求解器高效求解，复杂度从O(n³)降到接近O(n)。

以下类/函数片段摘自 [`cartographer_concepts.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/cartographer_concepts.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

本例残差为测量减预测，雅可比也必须针对该残差求导。修正原例混用预测雅可比造成的更新符号错误，graph_demo用有限差分核验每个雅可比元素并检查优化前后误差。固定第一个位姿，要求约束图连通。

```cpp
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
```

### 15.1.4 图优化 vs 滤波方法

| 特性 | 图优化 (GraphSLAM) | 滤波 (EKF-SLAM) |
|------|-------------------|-----------------|
| 状态维护 | 全部位姿+地图 | 当前位姿+地图 |
| 历史修正 | 可修正全部历史 | 不可修正 |
| 回环检测 | 自然支持 | 需要额外处理 |
| 计算复杂度 | O(n³)但可稀疏加速 | O(n²) |
| 实时性 | 批量优化 | 在线实时 |
| 精度 | 高（全优化） | 中（线性化误差） |
| 适用场景 | 大规模、有回环 | 小规模、低噪声 |

## 15.2 Cartographer系统架构

### 15.2.1 总体架构

Cartographer是Google开源的基于图优化的SLAM系统，支持2D和3D建图。其核心架构分为前端和后端两部分：

```
传感器数据流:
┌─────────┐  LaserScan  ┌──────────────────────┐
│ 激光雷达  │──────────→│                      │
└─────────┘            │    Local SLAM        │
┌─────────┐  Imu       │    (前端匹配)         │
│   IMU    │──────────→│    - 体素滤波         │
└─────────┘            │    - 扫描匹配         │
┌─────────┐  Odometry  │    - 子图插入         │
│ 里程计   │──────────→│                      │
└─────────┘            └──────────┬───────────┘
                                  │ Submaps
                                  ↓
┌─────────────────────────────────────────────┐
│         Global SLAM (后端优化)               │
│    - 回环检测 (分支定界加速)                  │
│    - 图优化 (Ceres Solver)                   │
│    - 全局一致的位姿和地图修正                  │
└─────────────────────────────────────────────┘
```

### 15.2.2 核心概念

**Submap（子图）** 是由连续多帧激光数据构成的局部栅格地图，每个子图包含一定数量的激光扫描帧，子图内部通过前端匹配实现局部一致性。

**Node（节点）** 对应关键帧位姿，每个子图插入时创建一个节点，节点包含位姿估计、时间戳和相关传感器数据。

**Constraint（约束）** 分为帧间约束和子图间约束两类：前者是同一子图内相邻帧的匹配，后者是不同子图之间的匹配（回环）；约束包含相对变换与信息矩阵。

以下类/函数片段摘自 [`teaching_fragments.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/teaching_fragments.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

这是流程声明而非完整程序。原CartographerCore缺少_compute_relative_transform、_fast_correlative_scan_match以及_last_timestamp/_last_scan维护；接口命名不代表实现已完成。这里保留10帧子图、100×100栅格、0.05 m分辨率、局部搜索范围和0.7回环阈值；实际Cartographer运行使用15.4节官方节点入口。

```cpp
// 流程声明：依赖尚未定义的历史子图锚点与约束生成规则，不能实例化为建图节点。
struct CartographerCore {
  int submap_size=10;
  // 1. 根据里程计、上一时间戳及历史位姿预测初值。
  // 2. 初值附近以 dx/dy∈[-0.1,0.1]、dtheta∈[-0.05,0.05] 各5档扫描匹配。
  // 3. 保存当前位姿，把扫描终点插入100×100、0.05 m子图。
  // 4. 生成相邻节点的相对位姿约束，信息矩阵100 I。
  // 5. 每10帧新建子图；历史子图匹配得分>0.7时考虑回环。
  // 原例未定义以下两项；须先定义其坐标系、锚点和输入输出，再补全实现。
  Vector3d compute_relative_transform(const Vector3d & previous,const Vector3d & current);
  Vector3d fast_correlative_scan_match(const Points & scan,const Vector3d & initial,int historical_submap);
};
```

### 15.2.3 Local SLAM前端流程

Cartographer的前端Local SLAM负责构建局部一致的子图：

```
传感器输入
    ↓
体素滤波 (Voxel Filter)
- 降低点云密度，去除离群点
- 自适应体素大小
    ↓
位姿外推器 (Pose Extrapolator)
- 基于匀速模型预测初始位姿
- 融合IMU数据（如有）
    ↓
扫描匹配 (Scan Match)
- Ceres Solver优化
- 最小化扫描与子图的差异
    ↓
运动滤波器 (Motion Filter)
- 时间/位移/角度阈值
- 筛选关键帧
    ↓
子图插入 (Submap Insertion)
- 将关键帧融入当前子图
- 更新子图占据概率
```

以下类/函数片段摘自 [`cartographer_concepts.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/cartographer_concepts.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

以下为前端原理教学：体素0.05 m、运动阈值0.3 m/30°/1 s、子图200×200/50帧均保留；原固定0.1 s外推假设明确保留。扫描匹配使用逐点Gauss–Newton正规方程，而不是原例将总梯度外积误作Hessian；补齐旋转导数、地图原点及sigmoid链式导数。没有调用Ceres，不能标作Ceres实现。

```cpp
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
```

## 15.3 全局SLAM后端优化

### 15.3.1 回环检测

Cartographer使用**分支定界法（Branch and Bound）**加速回环检测的搜索过程。

**回环检测流程：**
```
1. 当子图完成，进入后端优化队列
2. 对每个新节点，在所有已完成子图中搜索匹配
3. 使用分支定界法快速搜索最优匹配
4. 匹配得分超过阈值时，添加回环约束到图
5. 执行全局图优化
```

以下类/函数片段摘自 [`cartographer_concepts.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/cartographer_concepts.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

原FastCorrelativeScanMatcher实际是粗细两级枚举，没有分支定界，预计算积分图也未参与打分；此处按实际算法命名说明并保留3/3/0.5搜索范围和0.05/0.01步长。精细搜索固定在粗搜索最优点周围，避免中心在循环中漂移。原GlobalSLAM回环i、j均指向当前节点，形成自环，且没有定义历史子图与图节点锚点；其完整后端连接规则尚需明确，不作为可运行产品。

```cpp
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
```

### 15.3.2 Ceres Solver在Cartographer中的应用

Cartographer使用Google的Ceres Solver进行非线性最小二乘优化：

```cpp
// Cartographer中Ceres优化问题的构建（伪代码）
struct ScanMatchCostFunction {
    ScanMatchCostFunction(const Eigen::Vector2d& point,
                          const Grid2D& grid)
        : point_(point), grid_(grid) {}

    template <typename T>
    bool operator()(const T* const pose,
                    T* residual) const {
        // 变换激光点
        const T transformed_x = pose[0] + cos(pose[2]) * T(point_.x())
                                        - sin(pose[2]) * T(point_.y());
        const T transformed_y = pose[1] + sin(pose[2]) * T(point_.x())
                                        + cos(pose[2]) * T(point_.y());

        // 双线性插值获取地图占据概率
        const T occupancy = grid_.GetOccupancy(transformed_x, transformed_y);

        // 残差 = 1 - occupancy
        residual[0] = T(1.0) - occupancy;
        return true;
    }

private:
    Eigen::Vector2d point_;
    const Grid2D& grid_;
};
```

### 15.3.3 全局优化效果

全局优化的核心作用是消除累积误差：

```
优化前（漂移明显）:
┌───────────────────┐
│  x1 → x2 → x3     │
│       ↓          │
│       x4 → x5     │
│            ↓      │
│  ←--- 回环 ---→   │
│  误差积累 → 地图错位 │
└───────────────────┘

优化后（全局一致）:
┌───────────────────┐
│ x1 → x2 → x3     │
│  ↓    ↓    ↓     │
│ x4 → x5 → x1 ←---│  ← 回环闭合
│  所有约束均衡分布     │
└───────────────────┘
```

## 15.4 Cartographer配置与ROS2集成

### 15.4.1 LUA配置文件

Cartographer使用LUA脚本进行配置：

```lua
-- cartographer_2d.lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D = {
  min_range = 0.05,
  max_range = 8.0,
  min_z = -0.5,
  max_z = 1.5,
  missing_data_ray_length = 5.,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.05,

  adaptive_voxel_filter = {
    max_length = 0.5,
    min_num_points = 200,
    max_range = 8.,
  },

  use_imu_data = false,
  use_online_correlative_scan_matching = true,

  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,
    angular_search_window = math.rad(20.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    occupied_space_weight = 20.,
    translation_weight = 10.,
    rotation_weight = 40.,
    ceres_solver_options = {
      use_nonmonotonic_steps = true,
      max_num_iterations = 10,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 1.0,
    max_distance_meters = 0.3,
    max_angle_radians = math.rad(30.),
  },
}

POSE_GRAPH = {
  optimize_every_n_nodes = 90,
  constraint_builder = {
    sampling_ratio = 0.3,
    max_constraint_distance = 15.,
    min_score = 0.55,
    global_localization_min_score = 0.6,
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,

    log_residual_histogram = true,

    ceres_scan_matcher = {
      occupied_space_weight = 30.,
      translation_weight = 10.,
      rotation_weight = 40.,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },

    fast_correlative_scan_matcher = {
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,
    },
  },

  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,

  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1e3,
    rotation_weight = 3e5,
    local_slam_pose_translation_weight = 1e5,
    local_slam_pose_rotation_weight = 1e5,
    odometry_translation_weight = 1e5,
    odometry_rotation_weight = 1e5,
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    log_solver_summary = false,
    use_online_imu_extrinsics_in_3d = true,
    fix_z_in_3d = false,
  },

  max_num_final_iterations = 200,
  global_sampling_ratio = 0.003,
  log_residual_histogram = true,
  global_constraint_search_after_n_seconds = 10.,
}
```

### 15.4.2 ROS2启动配置

```bash
# 安装 Cartographer ROS 2
# 依赖由第10章隔离镜像提供，不在COM260宿主安装x86软件包
source /workspace/install/setup.bash

# 启动2D建图
ros2 launch slam_lab cartographer_mapping.launch.py \
  configuration_directory:="$(ros2 pkg prefix slam_lab)/share/slam_lab/config/cartographer" \
  configuration_basename:=burger_2d.lua \
  use_sim_time:=true

# 或使用自定义启动文件
```

完整 Launch：[`cartographer_teaching.launch.py`](../src_k3_com260_kit/slam_teaching_cpp/launch/cartographer_teaching.launch.py)。

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    configuration_directory = os.path.join(
        get_package_share_directory('slam_lab'), 'config', 'cartographer')
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', 'burger_2d.lua',
            ],
            parameters=[{'use_sim_time': True}],
            remappings=[('scan', '/scan'), ('odom', '/odom')],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            parameters=[{'use_sim_time': True}],
        ),
    ])
```

### 15.4.3 参数调优指南

**前端匹配参数：**

| 参数 | 说明 | 调优建议 |
|------|------|---------|
| voxel_filter_size | 体素滤波大小 | 0.05(室内)~0.2(室外) |
| ceres_scan_matcher.translation_weight | 平移匹配权重 | 增大提高平移精度 |
| ceres_scan_matcher.rotation_weight | 旋转匹配权重 | 增大提高旋转精度 |
| min_range/max_range | 激光有效范围 | 根据传感器型号设置 |
| use_online_correlative_scan_matching | 在线相关匹配 | 复杂环境启用 |

**后端优化参数：**

| 参数 | 说明 | 调优建议 |
|------|------|---------|
| optimize_every_n_nodes | 优化间隔 | 小值更频繁但计算量大 |
| min_score | 回环检测最低得分 | 0.5~0.7，太高漏检 |
| linear_search_window | 回环搜索窗口(m) | 大范围回环需要增大 |
| angular_search_window | 回环搜索窗口(rad) | 旋转不确定时增大 |
| branch_and_bound_depth | 分支定界深度 | 7~12，深度大速度快 |

### 15.4.4 多传感器融合配置

Cartographer支持激光+IMU+里程计的多传感器融合：

```lua
-- 启用IMU
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- 配置IMU参数
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.

-- 启用里程计
options.use_odometry = true

-- 多激光雷达配置
options.num_laser_scans = 2  -- 使用两个激光雷达

-- 配置每个激光雷达的topic映射
-- 通过remapping实现
```

```bash
# 启动多传感器配置（使用自定义 Lua 配置）
ros2 run cartographer_ros cartographer_node \
  -configuration_directory ./config \
  -configuration_basename multi_sensor_2d.lua

# 话题重映射
# 在launch文件中添加remappings
```

### 15.4.5 官方要点——官方文档先前端、后后端的调优哲学

Cartographer 官方文档给出了一条与本章 15.4.3 一致的调优顺序：先保证 Local SLAM（前端）稳定，再调 Global SLAM（后端）。前端首先调 `TRAJECTORY_BUILDER_2D.voxel_filter_size` 与 `min_range`/`max_range`，确认 `use_online_correlative_scan_matching` 是否必要（里程计好时关掉更稳）；后端先看 `optimize_every_n_nodes` 的优化耗时，再动 `POSE_GRAPH.constraint_builder.min_score` 与 `fast_correlative_scan_matcher` 的搜索窗口。文档强调一个易错点：`optimize_every_n_nodes = 0` 会关闭后端优化，这是前端调试方法；本课程的 Cartographer 版本采用 `TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }`，加载已有 pbstream 并保留后端匹配。参见[官方纯定位配置](https://google-cartographer-ros.readthedocs.io/en/latest/going_further.html#pure-localization-in-a-given-map)。

另一个官方强调的概念是 submap 与约束的关系：Local SLAM 产出 submap（每个由若干连续扫描拼接），后端通过 submap 之间的匹配建立约束。若前端 submap 内部已经漂移（表现为 RViz 中子图弯曲、错位），后端无论怎么调都无法挽救——"漂移先查前端"是 Cartographer 社区的第一原则。

### 15.4.6 官方要点——3D 建图与多传感器配置

针对练习第 6 题的多楼层 3D 建图，官方 backpack 3D 示例给出三条经验：第一，IMU 不可省略——3D 模式依赖 IMU 提供重力方向与旋转先验，`imu_gravity_time_constant` 通常取 10，IMU 安装位置需在 tracking_frame 标定准确；第二，3D 模式用 `TRAJECTORY_BUILDER_3D`，其 `num_accumulated_range_data` 决定多少帧点云累积一次匹配，配合 `num_laser_scans` 与 remapping 支持多雷达；第三，多楼层数据可通过多次轨迹（multiple trajectories）在同一 pbstream 中合并，楼层间不强行加回环约束，最后统一输出配准后的点云地图。

`map_builder.lua`/`trajectory_builder.lua` 的模块化 include 结构（本章 15.5.3 已见）意味着自定义配置只需覆盖差异项，其余继承官方默认——这是官方背包配置的标准实践，建议直接从 `cartographer_ros/configuration_files` 目录的示例复制起步。

## 15.5 建图实践与案例

### 15.5.1 完整建图工作流

先按本章末尾仿真实例准备 x86 课程容器，每条常驻命令使用独立终端。

**1. x86课程容器：启动仿真**

```bash
source /workspace/install/setup.bash
ros2 launch sim_demo sim_bringup.launch.py \
  world:="$(ros2 pkg prefix sim_demo)/share/sim_demo/worlds/museum.sdf" \
  sensor_profile:=teacher gui:=true rviz:=true
```

**2. 另一个x86课程容器终端：启动Cartographer**

```bash
source /workspace/install/setup.bash
ros2 launch slam_lab cartographer_mapping.launch.py use_sim_time:=true
```

**3. RViz观察**

仿真入口已经启动 RViz。在已有窗口添加 Map（`/map`）和 TF 显示项，将 Fixed Frame 设为 `map`，观察子图拼接。不要重复启动同一建图系统。

**4. COM260：控制机器人探索**

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**5. 停止运动后，在x86课程容器保存并转换地图**

```bash
source /workspace/install/setup.bash
mkdir -p /workspace/output/maps
# 单条轨迹第一次启动时编号为0；重启轨迹后须查询实际编号
ros2 run slam_lab save_cartographer_state 0 /workspace/output/maps/my_map.pbstream
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename /workspace/output/maps/my_map.pbstream \
  -map_filestem /workspace/output/maps/my_map
```

保存节点先结束指定轨迹再写入状态；路径属于 Cartographer 服务所在的 x86 容器。确认 `.pbstream`、`.pgm` 和 `.yaml` 均生成，保存后不要继续给已结束轨迹发送运动指令。

**6. 使用生成的地图**

停止 Cartographer 后，可按第14章启动 `slam_lab amcl_localization.launch.py map:=/workspace/output/maps/my_map.yaml` 检查定位。原课程后续导航目标保留到导航章节；本节只验证建图、保存和定位，不把仅启动导航进程当作导航通过。

### 15.5.2 建图质量评估

完整程序：[`cartographer_map_quality.cpp`](../src_k3_com260_kit/slam_teaching_cpp/src/cartographer_map_quality.cpp)。

修正原例/submap_list类型与数组字段：使用cartographer_ros_msgs/msg/SubmapList.submap；全未知地图时占据/空闲比例取0，避免除零。覆盖率是当前地图矩形中的已知栅格比例，不等同于真实场景覆盖率或精度证明。占据和空闲比例沿用原例对 `100` 和 `0` 的精确计数；Cartographer 输出的已知栅格可能是中间概率，此时两项比例为零仍可有非零覆盖率。

```cpp
#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cartographer_ros_msgs/msg/submap_list.hpp>
class CartographerMapQuality : public rclcpp::Node {
public:
  CartographerMapQuality() : Node("cartographer_map_quality") {
    map_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",10,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        ++map_count_; if (msg->data.empty()) { return; }
        const auto mapped=std::count_if(msg->data.begin(),msg->data.end(),[](int v){return v>=0;});
        const auto occupied=std::count(msg->data.begin(),msg->data.end(),100);
        const auto free=std::count(msg->data.begin(),msg->data.end(),0);
        const double coverage=100.0*mapped/msg->data.size();
        RCLCPP_INFO(get_logger(),"建图进度: %.1f%% (占据=%.1f%%, 空闲=%.1f%%)",coverage,
          mapped ? 100.0*occupied/mapped : 0.0,mapped ? 100.0*free/mapped : 0.0);
        if (coverage>90 && map_count_>100) { RCLCPP_INFO(get_logger(),"建图覆盖率达标，建议完成建图并保存地图"); }
      });
    submaps_=create_subscription<cartographer_ros_msgs::msg::SubmapList>("/submap_list",10,
      [this](cartographer_ros_msgs::msg::SubmapList::ConstSharedPtr msg) {
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"子图数量: %zu",msg->submap.size());
      });
  }
private:
  std::size_t map_count_=0;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_;
  rclcpp::Subscription<cartographer_ros_msgs::msg::SubmapList>::SharedPtr submaps_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<CartographerMapQuality>()); rclcpp::shutdown();
}
```

### 15.5.3 纯定位模式

在建图完成后，Cartographer也可以运行在纯定位模式。使用前面保存的 pbstream，按以下步骤加载已有地图并启动定位轨迹。

x86课程容器先复制本章配置，再建立纯定位差异文件：

```bash
mkdir -p /workspace/output/config
cp "$(ros2 pkg prefix slam_lab)/share/slam_lab/config/cartographer/burger_2d.lua" /workspace/output/config/
```

将下列内容保存为 `/workspace/output/config/localization.lua`：

```lua
include "burger_2d.lua"
TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
return options
```

这里继承完整模型、传感器和分辨率配置，限制定位轨迹保留的活动子图数量。课程镜像中的 Cartographer 2.0.9002 不接受旧的 `pure_localization` 布尔参数。原例的 `optimize_every_n_nodes = 0` 关闭的是后端优化，不能实现纯定位；本例保留基础配置中的后端频率，后续按实测调优。

```bash
# x86课程容器；保持仿真与里程计运行，先停止原Cartographer
ros2 run cartographer_ros cartographer_node \
  -configuration_directory /workspace/output/config \
  -configuration_basename localization.lua \
  -load_state_filename /workspace/output/maps/my_map.pbstream \
  --ros-args -p use_sim_time:=true
```


在另一个已加载课程环境的 x86 容器终端检查轨迹和坐标变换：

```bash
ros2 service call /get_trajectory_states cartographer_ros_msgs/srv/GetTrajectoryStates '{}'
ros2 run tf2_ros tf2_echo map base_link
```

加载单条已保存轨迹时，实测返回轨迹 `0` 为 `FROZEN (2)`、新轨迹 `1` 为 `ACTIVE (0)`，并持续发布 `map → base_link` 变换；检查完按 `Ctrl+C` 退出。该结果验证配置、状态加载和 TF 输出，不代表全局重定位已收敛或定位精度达标。原始输出见[纯定位运行记录](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-pure-localization.cast)。

### 15.5.4 大规模建图优化策略

对于大规模环境（>10000m²）的建图，需要特殊的优化策略。以下为参数差异示意，不能替换完整默认表；实际配置逐项覆盖对应字段，并以实测调优：

```lua
-- large_scale_mapping.lua
-- 大规模建图参数优化

TRAJECTORY_BUILDER_2D = {
  -- 增大体素滤波以降低计算量
  voxel_filter_size = 0.15,

  -- 降低关键帧插入频率
  motion_filter = {
    max_time_seconds = 3.0,
    max_distance_meters = 0.5,
    max_angle_radians = math.rad(45.),
  },

  ceres_scan_matcher = {
    -- 减少Ceres迭代次数
    ceres_solver_options = {
      max_num_iterations = 5,
    },
  },
}

POSE_GRAPH = {
  -- 降低后端优化频率
  optimize_every_n_nodes = 150,

  -- 增大回环搜索范围
  constraint_builder = {
    max_constraint_distance = 30.,
    fast_correlative_scan_matcher = {
      linear_search_window = 10.,
      angular_search_window = math.rad(45.),
    },
  },
}
```

### 15.5.5 常见问题排查

**问题1：建图漂移** 的排查思路是：检查激光频率是否足够（建议>5Hz），检查IMU标定是否正确，降低移动速度，增大 `ceres_scan_matcher` 的权重。

**问题2：回环未闭合**
```lua
-- 增强回环检测
POSE_GRAPH.constraint_builder.min_score = 0.4  -- 降低回环阈值
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.0
POSE_GRAPH.optimize_every_n_nodes = 30
```

**问题3：CPU负载过高**
```lua
-- 降低计算负载
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.1
POSE_GRAPH.optimize_every_n_nodes = 200
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
```

### 15.5.6 官方要点——pbstream 工作流与 assets_writer

本章 15.5.1 的保存流程对应 cartographer_ros 的官方数据流：`/write_state` 服务把当前全部状态（轨迹、约束、子图）序列化为 pbstream；`cartographer_pbstream_to_ros_map` 把 pbstream 栅格化为 nav2 可用的 pgm/yaml。官方文档还提供 `cartographer_assets_writer`（ROS 1 时代的 assets_writer_backpack_3d 等示例），可以对已录制的 bag 离线重新建图并导出点云——这在需要更换参数重新处理旧数据时非常有用，也是"录制 bag、离线迭代参数"这一纪律在 Cartographer 生态中的标准做法。

纯定位通过 `-load_state_filename` 加载 pbstream，并设置 `TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }`。`optimize_every_n_nodes = 0` 不能替代纯定位开关；后端匹配频率需依据实测调整。

### 15.5.7 官方要点——维护现状与工程选型建议

需要了解的现状：Cartographer 主仓库由 Cartographer 团队维护，cartographer_ros 的 ROS 2 移植在早期（Foxy/Eloquent）较为活跃，此后更新节奏放缓，但其文档与配置体系仍是最完整的开源图优化 SLAM 参考资料。社区（包括 slam_toolbox 的维护者）的常见建议是：2D 室内建图优先评估 slam_toolbox（纯 ROS 2 原生、迭代快），需要 3D、多传感器紧耦合或 PBStream 生态时选 Cartographer；两者并非互斥，用 Cartographer 的参数文档理解 `ceres_scan_matcher`、branch-and-bound 回环等概念后，再读 slam_toolbox 的同名概念会事半功倍。

## 课后练习

1. **原理题:** 阐述图优化SLAM的基本原理，说明为什么图优化能够消除SLAM中的累积误差，推导Gauss-Newton优化的核心公式。

2. **编程题:** 实现一个简化的位姿图优化系统，支持添加位姿节点、添加里程计约束和回环约束，使用Gauss-Newton法进行全局优化。

3. **分析题:** 分析Cartographer前后端分离设计的优势，说明Local SLAM和Global SLAM各自的作用和相互协作机制。

4. **配置题:** 编写Cartographer的LUA配置文件，使得在10000m²的仓库环境中实现高质量建图，包括前端匹配、后端优化和回环检测参数。

5. **操作题:** 描述完整的Cartographer建图工作流程：从启动仿真、开始建图、控制探索到最终保存地图并转换为PGM/YAML格式。

6. **设计题:** 某高层建筑需要3D建图，机器人配备3D激光雷达、IMU和轮式里程计。设计基于Cartographer 3D的建图方案，包括传感器配置、参数调优策略、多楼层建图方案和地图配准方法。

---

## 仿真结合实例（当前仓库）：记录 Cartographer 图优化所需传感器数据

### 目标与知识点对应

Cartographer 需要连续的激光、里程计和 TF 数据。本章使用 `slam_lab` 的 Cartographer 配置；先用 `sim_demo` 采集输入，再按15.5.1启动官方节点，理解局部子图和后端优化所依赖的数据关系。

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
ros2 bag record --use-sim-time -o /tmp/cartographer_input /scan /odom /tf /tf_static
# 运行约 10 秒后 Ctrl+C
ros2 bag info /tmp/cartographer_input
```

在独立 COM260 终端启动自动运动演示；先观察小车周边空间，按 Ctrl+C 停止：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run param_demo_cpp patrol_driver --ros-args -p linear_speed:=0.12 -p angular_speed:=0.45
```

此终端接替原 `drive:=true` 的自动运动：复用第6章 C++ `patrol_driver`，线速度0.12 m/s、角速度0.45 rad/s，替代原仿真入口默认5.0 m/s、1.5 rad/s。传感器/TF观察目标不变，实际速度与轨迹以里程计为准；不要同时运行键盘控制。

### 观察结果

RViz 中可看到机器人运动和激光扫描；bag 的时间序列可用于后续 Cartographer 离线配置与回环实验。

### 源码与边界

仿真入口位于 `src_k3_com260_kit/sim_demo/launch/sim_bringup.launch.py`，传感器桥配置位于 `src_k3_com260_kit/sim_demo/config/bridge.yaml`，RViz 配置位于 `src_k3_com260_kit/sim_demo/rviz/sensors.rviz`。

当前仓库没有 Cartographer 后端，不能把 `slam_toolbox` 的在线地图称为 Cartographer 图优化结果。

学习材料：
- Cartographer 官方文档（readthedocs）—— 算法架构与调优指南：https://cartographer.readthedocs.io/
- Google cartographer_ros 仓库 —— 配置文件与 assets_writer 示例：https://github.com/cartographer-project/cartographer_ros
- Cartographer 论文 —— Hess et al., "Real-Time Loop Closure in 2D LIDAR SLAM"（ICRA 2016）：https://research.google/pubs/pub45466/
- The Construct —— Cartographer 建图与定位课程：https://www.theconstructsim.com/
- slam_toolbox Wiki —— 与 Cartographer 对照的参数体系：https://github.com/SteveMacenski/slam_toolbox/wiki
