#include <map>
#include <yaml-cpp/yaml.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "recovery_demo_cpp/action_client.hpp"

int main(int argc, char ** argv) {
  init_actions(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("multi_robot_scheduler");
  using Navigate = nav2_msgs::action::NavigateToPose;
  using Client = rclcpp_action::Client<Navigate>;
  std::map<std::string, Client::SharedPtr> robots;
  for (const auto & name : {"robot1", "robot2", "robot3"}) {
    robots[name] = rclcpp_action::create_client<Navigate>(node, std::string("/") + name + "/navigate_to_pose");
  }
  int pending = 0;
  bool failed = false;
  try {
    const auto tasks = YAML::LoadFile(node->declare_parameter<std::string>("task_file", "tasks.yaml"))["tasks"];
    for (const auto & task : tasks) {
      const auto name = task["robot"].as<std::string>();
      if (!robots.count(name)) { continue; }
      auto client = robots.at(name);
      if (!client->wait_for_action_server(10s)) { failed = true; break; }
      Navigate::Goal goal;
      goal.pose.header.frame_id = "map";
      goal.pose.header.stamp = node->now();
      goal.pose.pose.position.x = task["x"].as<double>();
      goal.pose.pose.position.y = task["y"].as<double>();
      goal.pose.pose.orientation.w = 1;
      goal.behavior_tree = task["recovery_strategy"].as<std::string>("") == "aggressive"
        ? "aggressive_recovery.xml" : "default_navigation.xml";
      Client::SendGoalOptions options;
      options.goal_response_callback = [&, name](auto handle) {
        RCLCPP_INFO(node->get_logger(), "%s: %s", name.c_str(), handle ? "ACCEPTED" : "REJECTED");
        failed |= !handle;
        --pending;
      };
      ++pending;
      client->async_send_goal(goal, options);
      RCLCPP_INFO(node->get_logger(), "派发任务给 %s: (%.2f, %.2f)", name.c_str(),
        goal.pose.pose.position.x, goal.pose.pose.position.y);
    }
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (pending && !stop_requested && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(20ms);
    }
    failed |= pending != 0 || stop_requested;
  } catch (const YAML::Exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    failed = true;
  }
  rclcpp::shutdown();
  return failed ? 1 : 0;
}
