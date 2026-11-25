#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include "auto_explorer/frontiers.hpp"

using namespace std::chrono_literals;
volatile std::sig_atomic_t stopped=0;
class AutoExplorer : public rclcpp::Node {
  using Navigate=nav2_msgs::action::NavigateToPose;
  using Handle=rclcpp_action::ClientGoalHandle<Navigate>;
public:
  AutoExplorer() : Node("auto_explorer"), transforms(get_clock()), listener(transforms) {
    prefix=declare_parameter("map_prefix",std::string(std::getenv("HOME"))+"/maps/final_map");
    const auto file=declare_parameter("area_file",ament_index_cpp::get_package_share_directory("auto_explorer")+"/config/exploration_area.yaml");
    const auto meta=YAML::LoadFile(file);
    mask=cv::imread((std::filesystem::path(file).parent_path()/meta["image"].as<std::string>()).string(),cv::IMREAD_GRAYSCALE);
    resolution=meta["resolution"].as<double>();
    origin={meta["origin"][0].as<double>(),meta["origin"][1].as<double>()};
    if (mask.empty()) throw std::runtime_error("Cannot read exploration area");
    cells=cv::countNonZero(mask); area=cells*resolution*resolution;
    if (area<500.0) throw std::runtime_error("Explorable area must be at least 500 m2");
    commands=create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    scan=create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr){last_scan=now();});
    maps=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg){grid=msg; measure();});
    navigation=rclcpp_action::create_client<Navigate>(this,"/navigate_to_pose");
    RCLCPP_INFO(get_logger(),"可探索区域 %.3f m²；覆盖率须超过 95%%",area);
  }
  int run() {
    auto self=shared_from_this();
    int idle=0;
    while (rclcpp::ok() && !stopped) {
      rclcpp::spin_some(self);
      if (coverage>95.0) { stop(); return save_map()?0:1; }
      if (!grid || !navigation->action_server_is_ready() || !scan_fresh()) { rclcpp::sleep_for(200ms); continue; }
      exploration::Point robot;
      try {
        const auto tf=transforms.lookupTransform("map","base_link",tf2::TimePointZero);
        robot={tf.transform.translation.x,tf.transform.translation.y};
      } catch (const tf2::TransformException &) { rclcpp::sleep_for(200ms); continue; }
      exploration::Point target;
      if (!exploration::next_frontier(*grid,robot,visited,target)) {
        if (++idle>=30) { RCLCPP_ERROR(get_logger(),"无可达新边界，覆盖率 %.2f%%；未完成建图",coverage); stop(); return 1; }
        rclcpp::sleep_for(1s); continue;
      }
      idle=0;
      Navigate::Goal goal;
      goal.pose.header.frame_id="map"; goal.pose.header.stamp=now();
      goal.pose.pose.position.x=target.x; goal.pose.pose.position.y=target.y;
      const double angle=std::atan2(target.y-robot.y,target.x-robot.x);
      goal.pose.pose.orientation.z=std::sin(angle/2); goal.pose.pose.orientation.w=std::cos(angle/2);
      RCLCPP_INFO(get_logger(),"探索边界 (%.2f, %.2f)，覆盖率 %.2f%%",target.x,target.y,coverage);
      auto pending=navigation->async_send_goal(goal);
      while (!stopped && rclcpp::spin_until_future_complete(self,pending,200ms)==rclcpp::FutureReturnCode::TIMEOUT) {}
      if (pending.wait_for(0s)!=std::future_status::ready) { cancel(); break; }
      auto handle=pending.get();
      if (!handle) { visited.push_back(target); continue; }
      auto result=navigation->async_get_result(handle);
      const auto start=now();
      while (!stopped && coverage<=95.0 && scan_fresh() && (now()-start).seconds()<180.0) {
        if (rclcpp::spin_until_future_complete(self,result,200ms)==rclcpp::FutureReturnCode::SUCCESS) break;
      }
      if (result.wait_for(0s)!=std::future_status::ready) cancel();
      else RCLCPP_INFO(get_logger(),"边界导航结果: %d",int(result.get().code));
      stop(); visited.push_back(target);
      for (int i=0;i<10 && !stopped;++i) { rclcpp::spin_some(self); rclcpp::sleep_for(200ms); }
    }
    cancel(); stop(); return 130;
  }
  void halt() { cancel(); stop(); }
  void stop() { for (int i=0;i<5;++i) { commands->publish(geometry_msgs::msg::Twist()); rclcpp::sleep_for(100ms); } }
private:
  bool scan_fresh() { return last_scan.nanoseconds()>0 && (now()-last_scan).seconds()<2.0; }
  void cancel() {
    auto request=navigation->async_cancel_all_goals();
    rclcpp::spin_until_future_complete(shared_from_this(),request,5s);
  }
  void measure() {
    const auto & m=grid->info;
    if (!m.width || !m.height || m.resolution<=0) return;
    int known=0;
    for (int y=0;y<mask.rows;++y) for (int x=0;x<mask.cols;++x) {
      if (!mask.at<uchar>(mask.rows-1-y,x)) continue;
      const int gx=std::floor((origin.x+(x+0.5)*resolution-m.origin.position.x)/m.resolution);
      const int gy=std::floor((origin.y+(y+0.5)*resolution-m.origin.position.y)/m.resolution);
      if (gx>=0 && gy>=0 && gx<int(m.width) && gy<int(m.height) && grid->data[gy*m.width+gx]>=0) ++known;
    }
    coverage=100.0*known/cells;
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"场内覆盖率 %.3f%% (%d/%d)，面积 %.3f m²",coverage,known,cells,area);
  }
  bool save_map() {
    const auto path=std::filesystem::path(prefix);
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    std::ofstream image(prefix+".pgm",std::ios::binary), metadata(prefix+".yaml");
    const auto & info=grid->info;
    image<<"P5\n"<<info.width<<' '<<info.height<<"\n255\n";
    for (int y=int(info.height)-1;y>=0;--y) for (unsigned x=0;x<info.width;++x) {
      const int value=grid->data[y*info.width+x];
      image.put(value<0?205:(value>=65?0:(value<=25?254:205)));
    }
    metadata<<"image: "<<path.filename().string()<<".pgm\nresolution: "<<info.resolution
      <<"\norigin: ["<<info.origin.position.x<<", "<<info.origin.position.y
      <<", 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    image.flush(); metadata.flush();
    const bool ok=bool(image)&&bool(metadata);
    RCLCPP_INFO(get_logger(),"覆盖率 %.3f%%，地图保存%s: %s",coverage,ok?"成功":"失败",prefix.c_str());
    return ok;
  }
  std::string prefix;
  cv::Mat mask;
  exploration::Point origin;
  std::vector<exploration::Point> visited;
  double resolution=0,area=0,coverage=0;
  int cells=0;
  rclcpp::Time last_scan{0,0,RCL_ROS_TIME};
  tf2_ros::Buffer transforms;
  tf2_ros::TransformListener listener;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
  rclcpp_action::Client<Navigate>::SharedPtr navigation;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commands;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr maps;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,[](int){stopped=1;}); std::signal(SIGTERM,[](int){stopped=1;});
  int result=1;
  std::shared_ptr<AutoExplorer> node;
  try { node=std::make_shared<AutoExplorer>(); result=node->run(); }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("auto_explorer"),"%s",e.what());
    if (node) { node->halt(); }
  }
  rclcpp::shutdown(); return result;
}
