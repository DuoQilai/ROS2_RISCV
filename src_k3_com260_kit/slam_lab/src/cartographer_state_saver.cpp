#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/write_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cartographer_state_saver");
  int status = 0;
  try {
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() != 3) {throw std::invalid_argument("Usage: save_cartographer_state TRAJECTORY_ID SERVER_ABSOLUTE_PATH");}
    size_t used = 0;
    const int trajectory_id = std::stoi(args[1], &used);
    if (used != args[1].size() || trajectory_id < 0) {throw std::invalid_argument("Trajectory ID must be a nonnegative integer");}
    if (!std::filesystem::path(args[2]).is_absolute()) {throw std::invalid_argument("Output must be an absolute path on the Cartographer server");}
    auto finish = node->create_client<cartographer_ros_msgs::srv::FinishTrajectory>("/finish_trajectory");
    auto write = node->create_client<cartographer_ros_msgs::srv::WriteState>("/write_state");
    if (!finish->wait_for_service(10s)) {throw std::runtime_error("/finish_trajectory is unavailable");}
    if (!write->wait_for_service(10s)) {throw std::runtime_error("/write_state is unavailable");}
    auto finish_request = std::make_shared<cartographer_ros_msgs::srv::FinishTrajectory::Request>();
    finish_request->trajectory_id = trajectory_id;
    auto finish_future = finish->async_send_request(finish_request);
    if (rclcpp::spin_until_future_complete(node, finish_future, 30s) != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Failed to finish trajectory");
    }
    const auto finish_status = finish_future.get()->status;
    if (finish_status.code != 0) {throw std::runtime_error("Finish failed: " + finish_status.message);}
    auto write_request = std::make_shared<cartographer_ros_msgs::srv::WriteState::Request>();
    write_request->filename = args[2];
    write_request->include_unfinished_submaps = true;
    auto write_future = write->async_send_request(write_request);
    if (rclcpp::spin_until_future_complete(node, write_future, 30s) != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Failed to write Cartographer state");
    }
    const auto write_status = write_future.get()->status;
    if (write_status.code != 0) {throw std::runtime_error("Write failed: " + write_status.message);}
    RCLCPP_INFO(node->get_logger(), "Saved Cartographer state on server: %s", args[2].c_str());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
