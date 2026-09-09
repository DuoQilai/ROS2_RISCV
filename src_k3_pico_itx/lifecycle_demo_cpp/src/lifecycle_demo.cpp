#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleDemo : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleDemo()
  : LifecycleNode("hello_ros2_lifecycle"), count_(0)
  {
    const bool autostart = declare_parameter<bool>("autostart", false);
    RCLCPP_INFO(get_logger(), "LIFECYCLE_READY autostart=%s", autostart ? "true" : "false");
  }

  bool autostart()
  {
    if (!get_parameter("autostart").as_bool()) {
      return true;
    }
    return trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id() ==
           lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
           trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id() ==
           lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10).reliable().durability_volatile());
    timer_ = create_wall_timer(500ms, std::bind(&LifecycleDemo::publish, this));
    RCLCPP_INFO(get_logger(), "LIFECYCLE_CONFIGURED");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    publisher_->on_activate();
    RCLCPP_INFO(get_logger(), "LIFECYCLE_ACTIVE");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    publish_zero();
    publisher_->on_deactivate();
    RCLCPP_INFO(get_logger(), "LIFECYCLE_INACTIVE");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    publisher_.reset();
    count_ = 0;
    RCLCPP_INFO(get_logger(), "LIFECYCLE_CLEANED_UP");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    publish_zero();
    if (publisher_ && publisher_->is_activated()) {
      publisher_->on_deactivate();
    }
    RCLCPP_INFO(get_logger(), "LIFECYCLE_SHUTDOWN");
    return CallbackReturn::SUCCESS;
  }

private:
  void publish()
  {
    if (!publisher_ || !publisher_->is_activated()) {
      return;
    }
    geometry_msgs::msg::Twist message;
    message.linear.x = 0.1;
    publisher_->publish(message);
    ++count_;
    RCLCPP_INFO(get_logger(), "CMD_VEL_PUBLISHED count=%zu", count_);
  }

  void publish_zero()
  {
    if (publisher_ && publisher_->is_activated()) {
      publisher_->publish(geometry_msgs::msg::Twist());
      RCLCPP_INFO(get_logger(), "CMD_VEL_ZERO");
    }
  }

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleDemo>();
  if (!node->autostart()) {
    RCLCPP_ERROR(node->get_logger(), "LIFECYCLE_AUTOSTART_FAILED");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
