#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

using namespace std::chrono_literals;
volatile std::sig_atomic_t stopped = 0;

class NavGoalRunner : public rclcpp::Node {
  using Action = nav2_msgs::action::NavigateToPose;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
public:
  NavGoalRunner() : Node("nav_goal_runner") {
    client = rclcpp_action::create_client<Action>(this, "navigate_to_pose");
    initial = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    odom = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      x = msg->pose.pose.position.x; y = msg->pose.pose.position.y;
    });
  }
  bool prepare() {
    for (const auto & name : {"amcl", "bt_navigator", "planner_server", "controller_server"}) {
      auto service = create_client<lifecycle_msgs::srv::GetState>(std::string(name)+"/get_state");
      const auto deadline = std::chrono::steady_clock::now()+60s;
      bool active=false;
      while (!stopped && std::chrono::steady_clock::now()<deadline && !active) {
        if (!service->wait_for_service(1s)) { continue; }
        auto request = service->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
        active = rclcpp::spin_until_future_complete(shared_from_this(), request, 2s) ==
          rclcpp::FutureReturnCode::SUCCESS && request.get()->current_state.id == 3;
      }
      if (!active) { RCLCPP_ERROR(get_logger(), "%s 未激活", name); return false; }
    }
    if (!client->wait_for_action_server(10s)) { return false; }
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id="map"; pose.pose.pose.position.x=5.0;
    pose.pose.pose.orientation.z=std::sin(-1.0); pose.pose.pose.orientation.w=std::cos(-1.0);
    pose.pose.covariance[0]=0.25; pose.pose.covariance[7]=0.25; pose.pose.covariance[35]=0.0685;
    for (int i=0; i<5; ++i) {
      pose.header.stamp=now(); initial->publish(pose); rclcpp::spin_some(shared_from_this()); rclcpp::sleep_for(200ms);
    }
    RCLCPP_INFO(get_logger(), "Nav2 就绪，初始位姿: (5.0, 0.0, -2.0)");
    return true;
  }
  bool navigate(const std::array<double,3> & target) {
    if (stopped) { return false; }
    Action::Goal goal;
    goal.pose.header.frame_id="map"; goal.pose.header.stamp=now();
    goal.pose.pose.position.x=target[0]; goal.pose.pose.position.y=target[1];
    goal.pose.pose.orientation.z=std::sin(target[2]/2); goal.pose.pose.orientation.w=std::cos(target[2]/2);
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.feedback_callback=[this](Handle::SharedPtr, const auto feedback) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "距目标: %.2f m", feedback->distance_remaining);
    };
    auto sent=client->async_send_goal(goal,options);
    if (rclcpp::spin_until_future_complete(shared_from_this(),sent,10s)!=rclcpp::FutureReturnCode::SUCCESS || !sent.get()) {
      RCLCPP_ERROR(get_logger(), "目标被拒绝或响应超时"); return false;
    }
    auto result=client->async_get_result(sent.get());
    const auto start=std::chrono::steady_clock::now();
    const double start_x=x, start_y=y;
    bool canceled=false;
    auto cancel_time=start;
    while (result.wait_for(0s)!=std::future_status::ready) {
      rclcpp::spin_some(shared_from_this());
      const auto wall_now=std::chrono::steady_clock::now();
      if (!canceled && (stopped || wall_now-start>180s)) {
        client->async_cancel_goal(sent.get()); canceled=true; cancel_time=wall_now;
      }
      if (canceled && wall_now-cancel_time>10s) { return false; }
      rclcpp::sleep_for(20ms);
    }
    const auto code=result.get().code;
    RCLCPP_INFO(get_logger(), "结果: %s 耗时: %.1fs 位移: %.2fm",
      code==rclcpp_action::ResultCode::SUCCEEDED ? "SUCCEEDED" :
      code==rclcpp_action::ResultCode::CANCELED ? "CANCELED" : "ABORTED",
      std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(),std::hypot(x-start_x,y-start_y));
    return code==rclcpp_action::ResultCode::SUCCEEDED;
  }
private:
  double x=0, y=0;
  rclcpp_action::Client<Action>::SharedPtr client;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
};

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,[](int){stopped=1;}); std::signal(SIGTERM,[](int){stopped=1;});
  auto node=std::make_shared<NavGoalRunner>();
  const std::array<std::array<double,3>,4> goals{{{3,-1,-2},{2,1.5,1.57},{5,0.5,0},{1,-0.5,3.14}}};
  int count=0;
  if (node->prepare()) { for (const auto & goal:goals) { if (!node->navigate(goal)) { break; } ++count; } }
  RCLCPP_INFO(node->get_logger(),"导航完成: %d/4 个目标成功到达",count);
  rclcpp::shutdown();
  return count==4 ? 0 : (stopped ? 130 : 1);
}
