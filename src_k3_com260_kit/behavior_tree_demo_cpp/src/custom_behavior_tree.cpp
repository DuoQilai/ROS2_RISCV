#include <cmath>
#include <limits>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <std_msgs/msg/float32.hpp>
#include "recovery_demo_cpp/action_client.hpp"
#include "navigate.hpp"

int main(int argc, char ** argv) {
  init_actions(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("custom_behavior_tree");
  const double x = node->declare_parameter("x", 0.0);
  const double y = node->declare_parameter("y", 0.0);
  float battery = std::numeric_limits<float>::quiet_NaN();
  auto sub = node->create_subscription<std_msgs::msg::Float32>("/battery_status", 10,
    [&](const std_msgs::msg::Float32 & msg) { battery = msg.data; });
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!std::isfinite(battery) && !stop_requested && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(20ms);
  }
  BT::BehaviorTreeFactory factory;
  factory.registerSimpleCondition("BatteryCheck", [&](BT::TreeNode &) {
    RCLCPP_INFO(node->get_logger(), "电池电量: %.1f%%", battery * 100);
    return battery > 0.2f ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });
  factory.registerBuilder<Navigate>("Navigate", [&](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<Navigate>(name, config, node, x, y);
  });
  factory.registerSimpleAction("ClearCostmap", [&](BT::TreeNode &) {
    using Clear = nav2_msgs::srv::ClearEntireCostmap;
    if (stop_requested) { return BT::NodeStatus::FAILURE; }
    auto client = node->create_client<Clear>("/global_costmap/clear_entirely_global_costmap");
    if (!client->wait_for_service(5s)) { return BT::NodeStatus::FAILURE; }
    auto future = client->async_send_request(std::make_shared<Clear::Request>());
    return rclcpp::spin_until_future_complete(node, future, 5s) == rclcpp::FutureReturnCode::SUCCESS
      ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });
  auto tree = factory.createTreeFromText(R"(
    <root main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree">
      <Sequence name="MainSequence"><BatteryCheck/>
        <Fallback name="Navigation"><Navigate/>
          <Sequence name="RecoveryNav"><ClearCostmap/><Navigate/></Sequence>
        </Fallback>
      </Sequence>
    </BehaviorTree></root>)");
  auto previous = BT::NodeStatus::IDLE;
  while (rclcpp::ok() && !stop_requested) {
    rclcpp::spin_some(node);
    const auto status = tree.tickRoot();
    if (status != previous) {
      RCLCPP_INFO(node->get_logger(), "行为树结果: %s", BT::toStr(status).c_str());
      previous = status;
    }
    rclcpp::sleep_for(100ms);
  }
  tree.haltTree();
  rclcpp::shutdown();
  return 0;
}
