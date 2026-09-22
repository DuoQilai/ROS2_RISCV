#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DemoPublisher : public rclcpp::Node
{
public:
  DemoPublisher(const std::string & name, const std::string & topic,
    const std::string & label)
  : Node(name), label_(label)
  {
    publisher_ = create_publisher<std_msgs::msg::String>(topic, 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      std_msgs::msg::String message;
      message.data = label_ + "-" + std::to_string(count_++);
      publisher_->publish(message);
    });
  }

private:
  std::string label_;
  int count_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class DemoSubscriber : public rclcpp::Node
{
public:
  DemoSubscriber(const std::string & name, const std::string & topic,
    const std::string & label)
  : Node(name), label_(label)
  {
    group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    subscription_ = create_subscription<std_msgs::msg::String>(topic, 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        std::ostringstream thread;
        thread << std::this_thread::get_id();
        const auto start = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "%s 开始 thread=%s, msg=%s",
          label_.c_str(), thread.str().c_str(), message->data.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start).count();
        RCLCPP_INFO(get_logger(), "%s 结束 thread=%s, 用时=%.2fs",
          label_.c_str(), thread.str().c_str(), elapsed);
      }, options);
  }

private:
  std::string label_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::vector<rclcpp::Node::SharedPtr> nodes{
    std::make_shared<DemoPublisher>("publisher_a", "/multi_a", "PUB-A"),
    std::make_shared<DemoPublisher>("publisher_b", "/multi_b", "PUB-B"),
    std::make_shared<DemoSubscriber>("subscriber_a", "/multi_a", "SUB-A"),
    std::make_shared<DemoSubscriber>("subscriber_b", "/multi_b", "SUB-B")};
#ifdef USE_MULTI_EXECUTOR
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
#else
  rclcpp::executors::SingleThreadedExecutor executor;
#endif
  for (const auto & node : nodes) {
    executor.add_node(node);
  }
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
