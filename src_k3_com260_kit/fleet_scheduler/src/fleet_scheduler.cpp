#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <map>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using Navigate=nav2_msgs::action::NavigateToPose;
using Client=rclcpp_action::Client<Navigate>;
using Handle=rclcpp_action::ClientGoalHandle<Navigate>;
volatile std::sig_atomic_t stopped=0;

class FleetScheduler : public rclcpp::Node {
  struct Robot {Client::SharedPtr client; Handle::SharedPtr handle; bool busy=false,has_pose=false; double x=0,y=0;};
  struct Task {std::string robot; double x,y,yaw; int priority; bool dispatched=false,complete=false;};
public:
  FleetScheduler() : Node("multi_robot_scheduler") {
    for(const auto & name:{"robot1","robot2","robot3"}) {
      robots[name].client=rclcpp_action::create_client<Navigate>(this,std::string("/")+name+"/navigate_to_pose");
    }
    const auto data=YAML::LoadFile(declare_parameter<std::string>("task_file","tasks.yaml"));
    for(const auto & item:data["tasks"]) {
      const auto name=item["robot"].as<std::string>();
      if(!robots.count(name)) {throw std::runtime_error("Unknown robot: "+name);}
      tasks.push_back({name,item["x"].as<double>(),item["y"].as<double>(),item["yaw"].as<double>(0),item["priority"].as<int>(0)});
    }
    if(tasks.empty()) {throw std::runtime_error("Task file is empty");}
    std::stable_sort(tasks.begin(),tasks.end(),[](const Task & a,const Task & b){return a.priority>b.priority;});
    timer=create_wall_timer(1s,[this]{dispatch();});
  }
  bool done() const {return std::all_of(tasks.begin(),tasks.end(),[](const Task & t){return t.complete;});}
  bool failed=false;
  void cancel() {
    timer->cancel(); canceling=true;
    for(auto & [name,robot]:robots) {if(robot.handle) {robot.client->async_cancel_goal(robot.handle);}}
  }
  bool active() const {
    return std::any_of(robots.begin(),robots.end(),[](const auto & entry){return entry.second.busy;});
  }
private:
  void dispatch() {
    for(auto & task:tasks) {
      auto & robot=robots.at(task.robot);
      if(task.dispatched||robot.busy||!robot.client->action_server_is_ready()) {continue;}
      Navigate::Goal goal;
      goal.pose.header.frame_id="map"; goal.pose.header.stamp=now();
      goal.pose.pose.position.x=task.x; goal.pose.pose.position.y=task.y;
      goal.pose.pose.orientation.z=std::sin(task.yaw/2); goal.pose.pose.orientation.w=std::cos(task.yaw/2);
      robot.busy=true; task.dispatched=true;
      Client::SendGoalOptions options;
      options.goal_response_callback=[this,&task,&robot](Handle::SharedPtr handle) {
        robot.handle=handle;
        if(!handle) {robot.busy=false;task.complete=true;failed=true;}
        else if(canceling) {robot.client->async_cancel_goal(handle);}
        RCLCPP_INFO(get_logger(),"%s: %s",task.robot.c_str(),handle?"ACCEPTED":"REJECTED");
      };
      options.feedback_callback=[&robot](Handle::SharedPtr,auto feedback) {
        robot.x=feedback->current_pose.pose.position.x; robot.y=feedback->current_pose.pose.position.y;
        robot.has_pose=true;
      };
      options.result_callback=[this,&task,&robot](const Handle::WrappedResult & result) {
        const bool ok=result.code==rclcpp_action::ResultCode::SUCCEEDED;
        failed|=!ok; robot.busy=false; robot.handle.reset(); task.complete=true;
        RCLCPP_INFO(get_logger(),"%s: %s",task.robot.c_str(),ok?"SUCCEEDED":"FAILED_OR_CANCELED");
        if(robot.has_pose) {RCLCPP_INFO(get_logger(),"%s 反馈位置 (%.3f, %.3f)",task.robot.c_str(),robot.x,robot.y);}
      };
      robot.client->async_send_goal(goal,options);
      RCLCPP_INFO(get_logger(),"派发 %s priority=%d → (%.2f, %.2f)",task.robot.c_str(),task.priority,task.x,task.y);
    }
  }
  bool canceling=false;
  std::map<std::string,Robot> robots;
  std::vector<Task> tasks;
  rclcpp::TimerBase::SharedPtr timer;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,[](int){stopped=1;}); std::signal(SIGTERM,[](int){stopped=1;});
  int status=1;
  try {
    auto node=std::make_shared<FleetScheduler>();
    const auto deadline=std::chrono::steady_clock::now()+180s;
    while(!stopped&&!node->done()&&std::chrono::steady_clock::now()<deadline) {
      rclcpp::spin_some(node);rclcpp::sleep_for(20ms);
    }
    status=node->done()&&!node->failed?0:1;
    node->cancel();
    const auto cancel_deadline=std::chrono::steady_clock::now()+10s;
    while(node->active()&&std::chrono::steady_clock::now()<cancel_deadline) {rclcpp::spin_some(node);rclcpp::sleep_for(20ms);}
    if(stopped) {status=130;}
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';}
  rclcpp::shutdown(); return status;
}
