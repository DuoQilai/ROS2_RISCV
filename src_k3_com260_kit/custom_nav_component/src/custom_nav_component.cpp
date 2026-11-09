#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Nav2Component : public rclcpp_lifecycle::LifecycleNode {
public:
  Nav2Component() : LifecycleNode("custom_nav_component") {
    declare_parameter("param1", "default");
  }
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/custom_pose", 10);
    subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
        if (!publisher_->is_activated()) { return; }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "Active: scan=%zu frame=%s", scan->ranges.size(), scan->header.frame_id.c_str());
      });
    RCLCPP_INFO(get_logger(), "配置完成: param1=%s", get_parameter("param1").as_string().c_str());
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    publisher_->on_activate();
    RCLCPP_INFO(get_logger(), "已激活");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    publisher_->on_deactivate();
    RCLCPP_INFO(get_logger(), "已停用");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    subscriber_.reset();
    publisher_.reset();
    RCLCPP_INFO(get_logger(), "已清理");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override {
    on_cleanup(state);
    RCLCPP_INFO(get_logger(), "已关闭");
    return CallbackReturn::SUCCESS;
  }
private:
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Component>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}
