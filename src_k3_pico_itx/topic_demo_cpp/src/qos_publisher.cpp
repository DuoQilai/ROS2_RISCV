#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class QosPublisher : public rclcpp::Node
{
public:
  QosPublisher()
  : Node("qos_publisher"), count_(0)
  {
    rclcpp::QoS reliable(10);
    reliable.reliable();
    rclcpp::QoS best_effort(10);
    best_effort.best_effort();
    reliable_pub_ = create_publisher<std_msgs::msg::String>("/qos_reliable", reliable);
    best_effort_pub_ = create_publisher<std_msgs::msg::String>("/qos_best_effort", best_effort);
    timer_ = create_wall_timer(1s, std::bind(&QosPublisher::publish, this));
  }

private:
  void publish()
  {
    ++count_;
    std_msgs::msg::String reliable;
    reliable.data = "RELIABLE: " + std::to_string(count_);
    reliable_pub_->publish(reliable);
    std_msgs::msg::String best_effort;
    best_effort.data = "BEST_EFFORT: " + std::to_string(count_);
    best_effort_pub_->publish(best_effort);
    RCLCPP_INFO(get_logger(), "QOS_PUBLISHED count=%zu", count_);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reliable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr best_effort_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QosPublisher>());
  rclcpp::shutdown();
  return 0;
}
