#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <type_traits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using Waypoint=std::array<double,3>;
using Navigate=nav2_msgs::action::NavigateToPose;
using Follow=nav2_msgs::action::FollowWaypoints;
volatile std::sig_atomic_t stopped=0;

class NavigationConsole : public rclcpp::Node {
public:
  NavigationConsole() : Node("navigation_console") {
    declare_parameter<std::string>("mode","patrol");
    declare_parameter<std::string>("mission","");
    declare_parameter("loops",2);
    odom=create_subscription<nav_msgs::msg::Odometry>("/odom",10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & p=msg->pose.pose.position;
        if (has_position) { path_length+=std::hypot(p.x-x,p.y-y); }
        x=p.x; y=p.y; has_position=true;
      });
  }
  bool navigate(const Waypoint & point) {
    Navigate::Goal goal; goal.pose=pose(point);
    RCLCPP_INFO(get_logger(),"导航至: (%.2f, %.2f, %.1f°)",point[0],point[1],point[2]*180/M_PI);
    return execute<Navigate>("navigate_to_pose",goal);
  }
  bool follow(const std::vector<Waypoint> & points) {
    Follow::Goal goal;
    for(const auto & point:points) {goal.poses.push_back(pose(point));}
    return execute<Follow>("follow_waypoints",goal);
  }
  bool patrol(const std::vector<Waypoint> & points,int loops=3) {
    for(int round=0;round<loops&&!stopped;++round) {
      RCLCPP_INFO(get_logger(),"巡逻第 %d/%d 轮",round+1,loops);
      for(const auto & point:points) {if(!navigate(point)) {return false;}}
    }
    return !stopped;
  }
  bool mission(const std::string & file) {
    const auto data=YAML::LoadFile(file);
    RCLCPP_INFO(get_logger(),"加载任务: %s",data["name"].as<std::string>().c_str());
    for(const auto & task:data["tasks"]) {
      const auto type=task["type"].as<std::string>();
      if(type=="navigation") {
        if(!navigate({task["x"].as<double>(),task["y"].as<double>(),task["yaw"].as<double>(0)})) {return false;}
      } else if(type=="wait") {
        const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(task["duration"].as<double>());
        while(!stopped&&std::chrono::steady_clock::now()<end) {rclcpp::spin_some(shared_from_this());rclcpp::sleep_for(20ms);}
      } else if(type=="patrol") {
        std::vector<Waypoint> points;
        for(const auto & p:task["waypoints"]) {points.push_back(p.as<Waypoint>());}
        if(!patrol(points,task["loops"].as<int>(3))) {return false;}
      } else {throw std::runtime_error("Unknown mission task: "+type);}
      if(stopped) {return false;}
    }
    return true;
  }
  bool test(const std::vector<Waypoint> & points) {
    int successes=0;
    const auto start=std::chrono::steady_clock::now();
    for(const auto & point:points) {if(stopped) {break;} if(navigate(point)) {++successes;}}
    const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    RCLCPP_INFO(get_logger(),"测试: %d/%zu 成功，成功率 %.1f%%，平均耗时 %.2fs",
      successes,points.size(),100.0*successes/points.size(),elapsed/points.size());
    return successes==int(points.size());
  }
private:
  geometry_msgs::msg::PoseStamped pose(const Waypoint & point) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id="map"; p.header.stamp=now();
    p.pose.position.x=point[0]; p.pose.position.y=point[1];
    p.pose.orientation.z=std::sin(point[2]/2); p.pose.orientation.w=std::cos(point[2]/2);
    return p;
  }
  template<class Action> bool execute(const std::string & name,const typename Action::Goal & goal) {
    auto client=rclcpp_action::create_client<Action>(this,name);
    if(stopped||!client->wait_for_action_server(30s)) {return false;}
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.feedback_callback=[this](auto,auto feedback) {
      if constexpr(std::is_same_v<Action,Navigate>) {
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,"剩余距离: %.2fm",feedback->distance_remaining);
      } else {RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,"前往航点 #%u",feedback->current_waypoint);}
    };
    auto sent=client->async_send_goal(goal,options);
    if(rclcpp::spin_until_future_complete(shared_from_this(),sent,10s)!=rclcpp::FutureReturnCode::SUCCESS||!sent.get()) {return false;}
    auto result=client->async_get_result(sent.get());
    const auto start=std::chrono::steady_clock::now();
    const double initial_path=path_length;
    bool canceled=false;
    auto deadline=start+180s;
    while(result.wait_for(0s)!=std::future_status::ready) {
      rclcpp::spin_some(shared_from_this());
      if(!canceled&&(stopped||std::chrono::steady_clock::now()>deadline)) {
        client->async_cancel_goal(sent.get()); canceled=true; deadline=std::chrono::steady_clock::now()+10s;
      }
      if(canceled&&std::chrono::steady_clock::now()>deadline) {return false;}
      rclcpp::sleep_for(20ms);
    }
    const auto completed=result.get();
    bool ok=completed.code==rclcpp_action::ResultCode::SUCCEEDED;
    if constexpr(std::is_same_v<Action,Follow>) {ok=ok&&completed.result->missed_waypoints.empty();}
    RCLCPP_INFO(get_logger(),"导航%s，实际路径长度: %.2fm",ok?"成功":"失败或取消",path_length-initial_path);
    return ok;
  }
  double x=0,y=0,path_length=0;
  bool has_position=false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,[](int){stopped=1;}); std::signal(SIGTERM,[](int){stopped=1;});
  auto console=std::make_shared<NavigationConsole>();
  const std::vector<Waypoint> points{{1,.5,0},{2,1,M_PI/4},{1.5,2,M_PI/2},{0,1.5,M_PI}};
  bool ok=false;
  try {
    const auto mode=console->get_parameter("mode").as_string();
    if(mode=="patrol") {ok=console->patrol(points,console->get_parameter("loops").as_int());}
    else if(mode=="waypoints") {ok=console->follow(points);}
    else if(mode=="mission") {ok=console->mission(console->get_parameter("mission").as_string());}
    else if(mode=="test") {ok=console->test(points);}
    else {throw std::runtime_error("Use mode patrol, waypoints, mission or test");}
  } catch(const std::exception & e) {RCLCPP_ERROR(console->get_logger(),"%s",e.what());}
  rclcpp::shutdown(); return stopped?130:(ok?0:1);
}
