# 第14章 AMCL定位

本章为 COM260 Kit 的 C++17 教学副本。课程概念、练习及数值要求沿用原章；实测状态与素材见[实验第10章](../lab_manuals_k3_com260_kit/ch10_lab.md)和[运行证据](../lab_manuals_k3_com260_kit/runtime_evidence.md#ch10)。算法片段展示相应数学步骤，不能代替完整 SLAM 系统。

数值示例和 ROS 已知输入已分别在 COM260 与 x86 构建运行。原始记录：[COM260数值](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-numerics.cast)、[COM260 ROS接口](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-ros.cast)、[真实仿真传感器](../lab_manuals_k3_com260_kit/images/ch10/ch10-teaching-live-sensors.cast)。这些检查的输入和范围分开记录，不能据此宣称所有教学算法均在真实场景达到定位精度要求。

教学程序位于 [`slam_teaching_cpp`](../src_k3_com260_kit/slam_teaching_cpp/README.md)。COM260 终端先执行：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
```


> **课程**：ROS2 C++17 编程
> **章节**：第14章
> **课时**：2 课时（90 分钟）
> **教学方式**：讲授 + 演示

---

## 学习目标

本章学习目标包括：理解自适应蒙特卡洛定位(AMCL)的原理和算法流程，掌握粒子滤波定位的核心技术，熟悉KLD自适应采样和重采样策略，能够在ROS2中配置和使用AMCL进行机器人定位，掌握AMCL参数调优方法。

## 14.1 AMCL定位原理

### 14.1.1 定位问题定义

机器人定位是在已知地图的情况下，根据传感器数据估计自身位姿的过程。这是导航三大问题中的"我在哪里？"。

**定位问题的数学形式：**
```
p(x_t | z_{1:t}, u_{1:t-1}, m)
```

其中：`x_t` 是机器人在时刻 t 的位姿，`z_{1:t}` 是从起始到 t 时刻的所有观测，`u_{1:t-1}` 是从起始到 t-1 时刻的所有控制输入，`m` 是已知环境地图。

**定位的三种类型：**全局定位是机器人不知道初始位姿，需要在地图中定位自己；位置追踪是已知初始近似位姿，持续跟踪位置变化；绑架问题是机器人被意外移动到新位置，需要重新定位。

### 14.1.2 蒙特卡洛定位

蒙特卡洛定位 (Monte Carlo Localization, MCL) 使用粒子滤波实现机器人在已知地图上的定位。

**MCL核心思想：**使用一组加权粒子表示机器人位姿的后验分布；每个粒子是一个可能的位姿假设 (x, y, θ)；粒子权重表示该位姿与传感器观测的匹配程度。

以下类/函数片段摘自 [`localization.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/localization.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

随机初始化、运动dt=0.1和sigma=0.1保持原例。距离场教学实现使用精确穷举最近障碍物；只用于小地图数值演示，不声称达到KDTree性能。

```cpp
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
```

### 14.1.3 AMCL的核心改进

AMCL (Adaptive Monte Carlo Localization) 在标准MCL基础上增加了两个关键改进：

**KLD自适应采样：**根据粒子分布的熵动态调整粒子数量

**重采样策略改进：**结合短期和长期平均权重，检测和处理绑架问题

### 14.1.4 官方要点——ROS Wiki amcl 页面与绑架恢复的历史脉络

ROS Wiki 的 amcl 页面（ROS 1 遗产文档）解释了 `alpha_slow`/`alpha_fast` 的原始定义：两者分别是权重慢速/快速指数平均的衰减率，当快速平均低于慢速平均时（说明观测似然整体骤降，即被绑架或重定位失败），按两者比值插入随机粒子。本章 14.5.1 问题 3 的配置正源于此机制，Nav2 版本沿用了这一语义。

该页面同样记录了 `laser_max_beams` 的作用——把一次扫描均匀降采样为固定数量束参与似然计算。束数越多抗噪性越强但单次更新越慢，180～360 是常用区间；这与 14.5.1 问题 2「漂移时提高 beam 数」的建议互为印证。

## 14.2 粒子滤波定位详解

### 14.2.1 运动模型

AMCL支持两种运动模型：

**速度运动模型 (Velocity Motion Model)：**
```
x_t = f(x_{t-1}, u_t) + noise
```

其中：
```
x_t.x = x_{t-1}.x + v/ω · sin(θ + ω·Δt) - v/ω · sin(θ)
x_t.y = x_{t-1}.y - v/ω · cos(θ + ω·Δt) + v/ω · cos(θ)
x_t.θ = x_{t-1}.θ + ω·Δt + γ·Δt
```

**里程计运动模型 (Odometry Motion Model)：**
直接使用里程计测量的相对运动作为控制输入。

### 14.2.2 观测模型

AMCL支持两种传感器模型：

**波束模型 (Beam Model)：**
综合考虑四种测量情况：
1. 正确测量（高斯噪声）
2. 意外障碍物（指数分布）
3. 测量失败（均匀分布）
4. 随机测量（均匀分布）

以下类/函数片段摘自 [`localization.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/localization.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
inline double beam_model_likelihood(double z, double expected, double max_range,
  double zhit=0.95, double zshort=0.01, double zmax=0.01, double zrand=0.03) {
  const double sigma=0.1;
  const double hit=zhit*std::exp(-(z-expected)*(z-expected)/(2*sigma*sigma))/(std::sqrt(2*std::acos(-1.0))*sigma);
  const double short_return=z<expected ? zshort*2.0/expected*(1.0-z/expected) : 0.0;
  return hit+short_return+(z>=max_range-0.01 ? zmax : 0.0)+zrand/max_range;
}
```

**似然域模型 (Likelihood Field Model):**
计算激光点末端到地图障碍物的距离，转为概率。

以下类/函数片段摘自 [`localization.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/localization.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
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
```

### 14.2.3 重采样策略

**低方差重采样 (Low Variance Resampling)：**
以下类/函数片段摘自 [`localization.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/localization.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。



```cpp
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
```

## 14.3 KLD自适应采样

### 14.3.1 KLD采样原理

KLD (Kullback-Leibler Distance) 采样根据粒子分布的熵动态调整粒子数量：粒子发散时（不确定性高）增加粒子数，粒子收敛时（不确定性低）减少粒子数。

**KLD采样公式：**
```
n = (k-1) / (2·ε) · z²_{1-δ}
```

其中：`k` 是粒子分布覆盖的栅格数（离散化位姿空间），`ε` 是近似误差阈值，`z_{1-δ}` 是标准正态分布的 1-δ 分位数，`n` 是所需粒子数。

以下类/函数片段摘自 [`localization.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/localization.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

此处保留原例的简化粒子数公式及2.57常数，只作为自适应采样教学；delta参数在该近似式中未参与计算，不能据此宣称实现Nav2 AMCL的完整KLD界。

```cpp
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
```

### 14.3.2 官方要点——KLD 采样的信息论出处与实验建议

本章 14.3 的 KLD 自适应采样出自 Fox 与 Dieter Fox 团队的 KLD-Sampling 工作（蒙特卡洛定位系列论文），其核心结论是：所需样本数只与粒子分布覆盖的"有效栅格数 k"相关，收敛后 k 急剧缩小，因此可以安全降粒子数。文献中的实验表明，在约 1000 ㎡的办公楼里，KLD 采样能把平均粒子数从数千降到数百而保持同等定位精度。

按 The Construct 课程的建议复现：分别在开阔大厅与狭窄走廊录制 bag，观察 `/particle_cloud` 数量的动态变化；开阔处粒子应明显变少。这可以直接验证 14.5.2 的收敛过程示意，也是课后练习 4 的天然素材。

## 14.4 ROS2 AMCL配置

### 14.4.1 Nav2 AMCL节点

在ROS2中，AMCL是Nav2框架的一部分：

```bash
# x86课程容器：启动AMCL定位
source /workspace/install/setup.bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/workspace/output/maps/my_map.yaml \
  params_file:="$(ros2 pkg prefix slam_lab)/share/slam_lab/config/nav2_localization.yaml" \
  use_sim_time:=true
```

### 14.4.2 AMCL参数配置

```yaml
amcl:
  ros__parameters:
    # 粒子滤波参数
    max_particles: 2000
    min_particles: 500

    # 更新阈值
    update_min_a: 0.2       # 最小旋转更新(rad)
    update_min_d: 0.1       # 最小平移更新(m)

    # 激光模型
    laser_model_type: "likelihood_field"  # 或 "beam"
    laser_min_range: 0.1
    laser_max_range: 8.0
    max_beams: 180

    # 似然域模型参数
    z_hit: 0.95
    z_rand: 0.05
    sigma_hit: 0.2

    # 波束模型参数
    lambda_short: 0.1
    z_short: 0.01
    z_max: 0.01

    # 初始位姿
    initial_pose.x: 0.0
    initial_pose.y: 0.0
    initial_pose.yaw: 0.0

    # TF坐标系
    base_frame_id: "base_link"
    global_frame_id: "map"
    odom_frame_id: "odom"

    # 重采样参数
    resample_interval: 1   # 每次更新都重采样
    recovery_alpha_slow: 0.001      # 慢速平均权重衰减
    recovery_alpha_fast: 0.1        # 快速平均权重衰减

    # KLD参数
    pf_err: 0.05
    pf_z: 2.326347874  # δ=0.01 对应的标准正态 0.99 分位数

    # 变换
    set_initial_pose: true
    always_reset_initial_pose: false
    first_map_only: false
```

Humble Nav2 1.1.20 使用上述参数名。`pf_z` 直接参与 KLD 上界计算，不是失败概率 `δ`；这里将原 `δ=0.01` 换算为 `z₀.₉₉≈2.326347874`，区别于该版本默认值 `pf_z=0.99`。初值的不确定性通过 `/initialpose` 消息的 `pose.covariance` 设置：数组下标 0、7、35 分别是 x、y、yaw 方差，原例的三个 0.5 应写入这三个位置，单位分别为 m²、m²、rad²；可参照 14.5.1 的发布方法。参数和初值处理见 [Nav2 1.1.20 AMCL 源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_amcl/src/amcl_node.cpp)，KLD 计算见 [粒子滤波实现](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_amcl/src/pf/pf.c)。

### 14.4.3 AMCL启动与交互

完整程序：[`amcl_interface.cpp`](../src_k3_com260_kit/slam_teaching_cpp/src/amcl_interface.cpp)。

目标Humble Nav2 1.1.20的粒子消息为nav2_msgs/msg/ParticleCloud（particles数组，各项pose和weight），不能使用原例的PoseWithCovarianceStamped。本例订阅/particle_cloud，采用传感器QoS；以当次话题列表确认命名。可用amcl_interface x y yaw调用原set_initial_pose功能，等待订阅者就绪后发送一次，避免初始化消息在发现前丢失。

```cpp
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
class AMCLInterface : public rclcpp::Node {
public:
  AMCLInterface() : Node("amcl_interface") {
    initial_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10);
    pose_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose",10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        current_=*msg; const auto & p=msg->pose.pose; const auto & q=p.orientation;
        double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"AMCL位姿: (%.3f, %.3f, %.1f°) 不确定度=%.4f",
          p.position.x,p.position.y,yaw*180/std::acos(-1.0),std::sqrt(msg->pose.covariance[0]+msg->pose.covariance[7]));
      });
    particles_=create_subscription<nav2_msgs::msg::ParticleCloud>("/particle_cloud",rclcpp::SensorDataQoS(),
      [this](nav2_msgs::msg::ParticleCloud::ConstSharedPtr msg) { cloud_=*msg; });
    timer_=create_wall_timer(std::chrono::seconds(1),[this](){
      if (requested_ && initial_->get_subscription_count()>0) {
        initial_->publish(*requested_); requested_.reset(); RCLCPP_INFO(get_logger(),"初始位姿已发布");
      }
      if (!current_) { RCLCPP_INFO(get_logger(),"AMCL未初始化"); return; }
      const double quality=get_localization_quality();
      RCLCPP_INFO(get_logger(),"%s (质量=%.2f)",quality>0.7 ? "定位良好" : quality>0.3 ? "定位一般" : "定位差，建议重新初始化",quality);
    });
  }
  void set_initial_pose(double x,double y,double yaw) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id="map"; msg.header.stamp=now(); msg.pose.pose.position.x=x; msg.pose.pose.position.y=y;
    msg.pose.pose.orientation.z=std::sin(yaw/2); msg.pose.pose.orientation.w=std::cos(yaw/2);
    msg.pose.covariance[0]=0.25; msg.pose.covariance[7]=0.25; msg.pose.covariance[35]=0.068;
    requested_=msg;
  }
  double get_localization_quality() const {
    if (!current_ || cloud_.particles.size()<10) { return 0; }
    double x=0,y=0; for (const auto & p:cloud_.particles) { x+=p.pose.position.x; y+=p.pose.position.y; }
    x/=cloud_.particles.size(); y/=cloud_.particles.size(); double vx=0,vy=0;
    for (const auto & p:cloud_.particles) {
      vx+=(p.pose.position.x-x)*(p.pose.position.x-x); vy+=(p.pose.position.y-y)*(p.pose.position.y-y);
    }
    double spread=(std::sqrt(vx/cloud_.particles.size())+std::sqrt(vy/cloud_.particles.size()))/2;
    return std::clamp(1.0/(1.0+(current_->pose.covariance[0]+current_->pose.covariance[7])*10+spread*5),0.0,1.0);
  }
private:
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> current_,requested_;
  nav2_msgs::msg::ParticleCloud cloud_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_;
  rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr particles_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv); auto args=rclcpp::remove_ros_arguments(argc,argv);
  auto node=std::make_shared<AMCLInterface>();
  try {
    if (args.size()==4) {
      double values[3];
      for (std::size_t i=0;i<3;++i) {
        std::size_t used=0; values[i]=std::stod(args[i+1],&used);
        if (used!=args[i+1].size() || !std::isfinite(values[i])) { throw std::invalid_argument("位姿必须为有限数值"); }
      }
      node->set_initial_pose(values[0],values[1],values[2]);
    } else if (args.size()!=1) { throw std::invalid_argument("用法: amcl_interface [x y yaw]"); }
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(),"%s",error.what());rclcpp::shutdown();return 2;
  }
  rclcpp::shutdown();
}
```

### 14.4.4 定位性能监控

完整程序：[`amcl_monitor.cpp`](../src_k3_com260_kit/slam_teaching_cpp/src/amcl_monitor.cpp)。

粒子云消息按Humble实际类型修正；保留最近100项统计，缺少粒子或位姿时不计算空数组均值。cov_trace沿用原文x/y/z三个位置分量，不含yaw。

```cpp
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
class AMCLMonitor : public rclcpp::Node {
public:
  AMCLMonitor() : Node("amcl_monitor") {
    cloud_=create_subscription<nav2_msgs::msg::ParticleCloud>("/particle_cloud",rclcpp::SensorDataQoS(),
      [this](nav2_msgs::msg::ParticleCloud::ConstSharedPtr msg) {
        if (msg->particles.empty()) { return; }
        double x=0,y=0; for (const auto & p:msg->particles) { x+=p.pose.position.x;y+=p.pose.position.y; }
        x/=msg->particles.size();y/=msg->particles.size();double vx=0,vy=0;
        for (const auto & p:msg->particles) { vx+=std::pow(p.pose.position.x-x,2);vy+=std::pow(p.pose.position.y-y,2); }
        spreads_.push_back({std::sqrt(vx/msg->particles.size()),std::sqrt(vy/msg->particles.size())});
        if (spreads_.size()>100) { spreads_.pop_front(); }
      });
    pose_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose",10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        positions_.push_back({msg->pose.pose.position.x,msg->pose.pose.position.y});
        if (positions_.size()>100) { positions_.pop_front(); }
        covariance_=msg->pose.covariance[0]+msg->pose.covariance[7]+msg->pose.covariance[14];
      });
    timer_=create_wall_timer(std::chrono::seconds(1),[this](){
      if (positions_.empty() || spreads_.empty()) { return; }
      double sx=0,sy=0,x=0,y=0;
      for (const auto & p:spreads_) { sx+=p.first;sy+=p.second; }
      for (const auto & p:positions_) { x+=p.first;y+=p.second; }
      x/=positions_.size();y/=positions_.size();double vx=0,vy=0;
      for (const auto & p:positions_) { vx+=std::pow(p.first-x,2);vy+=std::pow(p.second-y,2); }
      RCLCPP_INFO(get_logger(),"cov_trace=%.6f avg_particle_spread=(%.6f,%.6f) position_variance=(%.6f,%.6f)",
        covariance_,sx/spreads_.size(),sy/spreads_.size(),vx/positions_.size(),vy/positions_.size());
    });
  }
private:
  std::deque<std::pair<double,double>> positions_,spreads_;double covariance_=0;
  rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr cloud_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<AMCLMonitor>());rclcpp::shutdown();
}
```

### 14.4.5 多传感器定位

AMCL可以融合多个传感器信息提高定位精度：

以下类/函数片段摘自 [`teaching_fragments.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/teaching_fragments.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

原MultiSensorAMCL缺少scan_callback、scan_back_callback和odom_callback，不能直接启动。这里保留已定义的IMU航向权重更新核心（sigma=0.05），不编造多传感器融合流程；完整节点属于后续设计练习。

```cpp
inline void apply_imu_heading(std::vector<MCLParticle> & particles,double imu_yaw) {
  for (auto & particle:particles) {
    double difference=std::abs(wrap(particle.pose.z()-imu_yaw));
    const double likelihood=std::exp(-difference*difference/(2*0.05*0.05));
    particle.weight*=likelihood;
  }
}
```

### 14.4.6 官方要点——nav2_amcl 官方文档的参数组与生命周期

按 Humble Nav2 1.1.20 的实际接口，可将参数分为通用（`scan_topic`、`set_initial_pose`、`always_reset_initial_pose`、`first_map_only`）、粒子滤波（`min_particles`、`max_particles`、`resample_interval`、`recovery_alpha_slow`/`recovery_alpha_fast`、`pf_err`/`pf_z`）、激光模型（`laser_model_type`、`max_beams`、`z_hit`/`z_rand`/`sigma_hit`、`lambda_short`）与运动模型。运动模型由 `robot_model_type` 选择，本课程差速模型使用 `nav2_amcl::DifferentialMotionModel`，对应 14.2.1 的差速运动模型。`update_min_d`/`update_min_a` 是触发滤波更新的移动距离/旋转角度阈值，过大会延后更新，过小会增加计算量。

作为 Nav2 生命周期的受管节点，AMCL 必须在 map_server 之后激活（依赖 `/map`）；`ros2 lifecycle set /amcl activate` 的顺序错误是新手定位失败的常见原因。官方教程还提供 `nav2_simple_commander` 中配合 initial pose 的标准初始化流程，可对照本章 14.4.3 的接口代码阅读。

## 14.5 定位问题与解决方案

### 14.5.1 常见定位问题

**问题1：全局定位失败**
```yaml
# 解决方案：增加允许的粒子数
max_particles: 5000
```

在 AMCL 已激活并收到地图后，向 `/initialpose` 发布初始位姿及较大的协方差。这里保持 x、y、yaw 方差均为 1.0；已知近似位置时应填写对应的 x/y/yaw，不应将未知位置直接当作原点。

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}, covariance: [1.0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0]}}"
```

**问题2：定位漂移**
```yaml
# 解决方案：强化观测模型
laser_model_type: "beam"  # 波束模型更精确
max_beams: 360  # 使用全部激光束
sigma_hit: 0.1  # 减小观测噪声
```

**问题3：绑架问题**
```yaml
# 解决方案：增加随机粒子
recovery_alpha_slow: 0.001
recovery_alpha_fast: 0.1
# 当快速平均权重远小于慢速平均权重时，AMCL会添加随机粒子
```

### 14.5.2 粒子收敛过程可视化

```
初始阶段（全局均匀分布）:
o o o o o o o o o o
o o o o o o o o o o
o o o o o o o o o o
o o o o o o o o o o
o o o o o o o o o o

运动后（粒子开始收敛）:
. . . . o . . . . .
. . . o o o . . . .
. . o o o o o . . .
. . . o o o . . . .
. . . . o . . . . .

收敛后（粒子聚拢）:
. . . . . . . . . .
. . . . . . . . . .
. . . o o o . . . .
. . . . . . . . . .
. . . . . . . . . .

成功定位后（单点估计）:
. . . . . . . . . .
. . . . × . . . . .
. . . . . . . . . .
```
(× 表示AMCL估计的最终位姿)

### 14.5.3 定位质量自动监控

以下类/函数片段摘自 [`teaching_fragments.hpp`](../src_k3_com260_kit/slam_teaching_cpp/include/slam_teaching_cpp/teaching_fragments.hpp)，使用该文件的头文件与 `slam_teaching` 命名空间。

原例缺少odom_callback以及last_known_pose的来源和可靠性规则。这里保留粒子数≥10、散布>2 m且连续超过10次的检测逻辑。自动重置只在可靠的map坐标初值已定义时才能实现；不能把odom坐标直接发布到map。恢复协方差(1.0,1.0,0.5)保持为设计目标，未发布未经定义的初始位姿。

```cpp
struct LocalizationHealthMonitor {
  int consecutive_bad_readings=0;bool auto_recovery_enabled=true;
  bool needs_recovery(const std::vector<Vector2d> & positions) {
    if (positions.size()<10) { return false; }
    Vector2d mean=Vector2d::Zero();for (const auto & p:positions) { mean+=p; }mean/=positions.size();
    Vector2d variance=Vector2d::Zero();
    for (const auto & p:positions) { variance+=(p-mean).array().square().matrix(); }
    const double spread=(variance/positions.size()).array().sqrt().mean();
    if (spread>2.0) { ++consecutive_bad_readings; }
    else { consecutive_bad_readings=0; }
    return consecutive_bad_readings>10 && auto_recovery_enabled;
  }
};
```

### 14.5.4 定位精度评估

```bash
# 检查TF连通性、发布频率与延迟；不能据此得出定位精度
ros2 run tf2_ros tf2_monitor

# 查看AMCL发布的位姿话题
ros2 topic echo /amcl_pose

# 查看粒子云
ros2 topic echo /particle_cloud

# RViz2可视化
rviz2
# 添加: ParticleCloud, PoseWithCovariance
```

### 14.5.5 官方要点——定位健康监控与失败恢复的工程纪律

Robotics Back-End 与 Nav2 文档共同强调三条定位健康纪律：第一，监控 `/amcl_pose` 协方差对角元素（尤其是 x、y 方差），方差持续增大说明粒子发散，应结合 `stddev_scan_diff`（本章 14.4.4 的思路）建立自动告警；第二，重定位失败时不要盲目重启节点，先用 RViz2 的 "2D Pose Estimate" 或发布 `/initialpose` 带大协方差试探性收敛，再逐步收紧；第三，`always_reset_initial_pose: false` 与 `first_map_only: false` 组合适用于多地图切换场景（练习第 6 题），而单地图固定起点部署可设 `set_initial_pose: true` 让启动即收敛。另外提醒：AMCL 只估计 `map`→`odom` 变换，航迹推算质量由 `odom_frame_id` 下游的里程计决定，定位异常时应先排除里程计漂移再调 AMCL 参数。

## 课后练习

1. **原理题:** 阐述AMCL中KLD自适应采样的原理，说明为什么在粒子收敛时可以减少粒子数而在发散时需要增加粒子数。

2. **编程题:** 实现一个简化版的AMCL系统，包含粒子初始化、运动模型预测、似然域权重更新和低方差重采样四个步骤。

3. **分析题:** 比较似然域模型和波束模型两种传感器模型的优缺点，说明在什么场景下应该选择哪种模型。

4. **配置题:** 在已知地图上启动AMCL定位，配置参数使得在以下场景中定位性能最优：a) 对称走廊 b) 开阔大厅 c) 狭窄通道。

5. **操作题:** 描述当AMCL定位丢失时，如何通过ROS2命令行手动重置初始位姿并恢复定位。

6. **设计题:** 某物流机器人需要在多个楼层间自动切换定位，设计一个跨楼层定位方案，包括地图切换机制、AMCL参数自适应调整和定位质量评估方法。

---

## 仿真结合实例（当前仓库）：预置地图上的 AMCL 初始位姿

### 目标与知识点对应

AMCL 需要地图、激光、里程计和初始位姿。本实例通过 `slam_lab` 启动 `map_server` 与 `amcl`，然后观察初始位姿发布和 `/amcl_pose` 输出。

### 运行步骤

仿真启动在 **x86 课程容器终端**执行，其他观测命令在 **COM260** 执行。x86 宿主仓库根目录先运行（每轮使用不同 RUN_ID）：

```bash
RUN_ID=teaching-slam
bash course_support/k3_com260_kit/scripts/x86-ch09-ch10.bash start "$RUN_ID"
bash course_support/k3_com260_kit/scripts/x86-ch09-ch10.bash exec "$RUN_ID" bash
```

容器终端已经加载 `/workspace/install/setup.bash`。按代码块标明的主机分别启动、观测，完成后先 Ctrl+C 停止节点，再在 x86 宿主执行 `bash course_support/k3_com260_kit/scripts/x86-ch09-ch10.bash stop "$RUN_ID"`。每次只运行一个建图/定位系统，防止多个节点同时发布 `map→odom`。


先启动与建图相同的 Museum 世界，使用独立 x86 课程容器终端：

```bash
source /workspace/install/setup.bash
ros2 launch sim_demo sim_bringup.launch.py \
  world:="$(ros2 pkg prefix sim_demo)/share/sim_demo/worlds/museum.sdf" \
  sensor_profile:=teacher gui:=true rviz:=false
```

另一个 x86 课程容器终端启动定位（先停止原建图节点）：

```bash
# x86 课程容器：先按实验第10章建图并保存为 /workspace/output/maps/my_map.yaml
source /workspace/install/setup.bash
ros2 launch slam_lab amcl_localization.launch.py \
  map:=/workspace/output/maps/my_map.yaml use_sim_time:=true use_rviz:=true
```

在 COM260 独立终端先启动本章两个观测节点，再发送初始位姿；它们分别持续运行：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run slam_teaching_cpp amcl_interface --ros-args -p use_sim_time:=true
```

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run slam_teaching_cpp amcl_monitor --ros-args -p use_sim_time:=true
```

当前 Humble 镜像中的 `/amcl_pose` 是 reliable + transient_local，而教学监控器沿用 volatile 订阅。晚启动且小车静止时会等待下一条位姿；先开监控再发初值，或让机器人产生新观测。等待新消息不等于 AMCL 未启动。RViz 粒子显示使用 `nav2_rviz_plugins/ParticleCloud`，对应 `/particle_cloud` 的 `nav2_msgs/msg/ParticleCloud`，Reliability 设为 `Best Effort`；Map 的 Durability 设为 `Transient Local`，便于晚加入的 RViz 收到静态地图。

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run slam_lab set_initial_pose 0.0 0.0 0.0 --ros-args -p use_sim_time:=true
ros2 topic echo /amcl_pose --once
ros2 topic echo /particle_cloud --once
ros2 topic info /map
```

### 观察结果

RViz 可显示地图、LaserScan 和 AMCL 位姿；用 `set_initial_pose X Y YAW_DEG` 调整初始位姿后，比较初始估计对定位过程的影响。

### 源码与边界

Nav2 Launch 位于 `src_k3_com260_kit/slam_lab/launch/amcl_localization.launch.py`，初始位姿节点位于 `src_k3_com260_kit/slam_lab/src/initial_pose_setter.cpp`；使用实验第10章实际保存的地图，不借用其他车型地图。

粒子云是否收敛应以本地 RViz 和 `/amcl_pose` 实际输出判断；启动日志本身不等于定位精度验证。

学习材料：
- docs.ros.org —— nav2_amcl 参数与节点文档：https://docs.ros.org/en/humble/p/nav2_amcl/
- Nav2 官方文档 —— Navigation System 与定位教程：https://docs.nav2.org/
- ROS Wiki —— amcl 包文档（历史参数语义）：https://wiki.ros.org/amcl
- The Construct —— ROS 2 Localization with AMCL 课程：https://www.theconstructsim.com/
- Robotics Back-End —— AMCL 定位实战与调参指南：https://roboticsbackend.com/
