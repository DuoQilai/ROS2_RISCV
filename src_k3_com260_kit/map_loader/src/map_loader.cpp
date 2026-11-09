#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/srv/load_map.hpp>

// PGM permits comments between header tokens and, in P2, between pixel values.
std::string token(std::istream & input) {
  std::string value;
  while (input >> std::ws && input.peek() == '#') {
    input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  if (!(input >> value)) { throw std::runtime_error("PGM 内容不完整"); }
  return value;
}

nav_msgs::msg::OccupancyGrid load_map(const std::string & path) {
  auto meta = YAML::LoadFile(path);
  auto image = std::filesystem::path(meta["image"].as<std::string>());
  if (image.is_relative()) { image = std::filesystem::path(path).parent_path() / image; }
  std::ifstream input(image, std::ios::binary);
  const auto format = token(input);
  const int width = std::stoi(token(input)), height = std::stoi(token(input));
  const int maximum = std::stoi(token(input));
  if ((format != "P2" && format != "P5") || width <= 0 || height <= 0 || maximum != 255) {
    throw std::runtime_error("示例支持最大灰度为 255 的 P2/P5 PGM");
  }
  if (format == "P5") {
    const char separator = input.get();
    if (separator == '\r' && input.peek() == '\n') { input.get(); }
  }
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = width;
  map.info.height = height;
  map.info.resolution = meta["resolution"].as<double>();
  map.info.origin.position.x = meta["origin"][0].as<double>();
  map.info.origin.position.y = meta["origin"][1].as<double>();
  const double yaw = meta["origin"][2].as<double>();
  map.info.origin.orientation.z = std::sin(yaw / 2);
  map.info.origin.orientation.w = std::cos(yaw / 2);
  const double occupied = meta["occupied_thresh"].as<double>(0.65);
  const double free = meta["free_thresh"].as<double>(0.196);
  const bool negate = meta["negate"].as<int>(0) != 0;
  if (!std::isfinite(map.info.resolution) || map.info.resolution <= 0 ||
      !std::isfinite(free) || !std::isfinite(occupied) || free < 0 || free >= occupied || occupied > 1 ||
      !std::isfinite(map.info.origin.position.x) || !std::isfinite(map.info.origin.position.y) || !std::isfinite(yaw)) {
    throw std::runtime_error("地图分辨率或阈值无效");
  }
  map.data.resize(width * height);
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      int pixel = format == "P2" ? std::stoi(token(input)) : input.get();
      if (pixel < 0 || pixel > 255) { throw std::runtime_error("PGM 像素无效或缺失"); }
      const double probability = (negate ? pixel : 255 - pixel) / 255.0;
      map.data[(height - row - 1) * width + col] = probability > occupied ? 100 : probability < free ? 0 : -1;
    }
  }
  return map;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_loader");
  auto publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());
  using Load = nav2_msgs::srv::LoadMap;
  auto service = node->create_service<Load>("/map_server/load_map",
    [&](Load::Request::SharedPtr request, Load::Response::SharedPtr response) {
      try {
        response->map = load_map(request->map_url);
        response->map.header.stamp = node->now();
        response->map.info.map_load_time = response->map.header.stamp;
        response->result = Load::Response::RESULT_SUCCESS;
        publisher->publish(response->map);
        RCLCPP_INFO(node->get_logger(), "地图已加载: %s (%ux%u)", request->map_url.c_str(),
          response->map.info.width, response->map.info.height);
      } catch (const std::exception & error) {
        response->result = Load::Response::RESULT_UNDEFINED_FAILURE;
        RCLCPP_ERROR(node->get_logger(), "地图加载失败: %s", error.what());
      }
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
}
