#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class CameraProcessor : public rclcpp::Node {
public:
  CameraProcessor() : Node("camera_processor") {
    image_sub = create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", rclcpp::QoS(10),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        latest_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "RGB: %dx%d frame=%s",
          latest_image.cols, latest_image.rows, msg->header.frame_id.c_str());
      });
    depth_sub = create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_raw", rclcpp::QoS(10),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        latest_depth = cv_bridge::toCvCopy(msg, "32FC1")->image;
        get_rgbd_pointcloud(msg->header);
      });
    info_sub = create_subscription<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", rclcpp::QoS(10),
      [this](const sensor_msgs::msg::CameraInfo & msg) {
        K = msg.k;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "相机内参: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
          K[0], K[4], K[2], K[5]);
      });
    cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/camera/points_sampled", 1);
  }
  void get_rgbd_pointcloud(const std_msgs::msg::Header & header) {
    if (latest_image.empty() || latest_depth.empty() || K[0] <= 0 || K[4] <= 0 ||
        latest_image.size() != latest_depth.size()) { return; }
    std::vector<cv::Vec3f> points;
    std::vector<cv::Vec3b> colors;
    for (int v = 0; v < latest_depth.rows; v += 5) {
      for (int u = 0; u < latest_depth.cols; u += 5) {
        float z = latest_depth.at<float>(v, u);
        if (!std::isfinite(z) || z <= 0 || z > 5) { continue; }
        points.push_back({float((u - K[2]) * z / K[0]), float((v - K[5]) * z / K[4]), z});
        colors.push_back(latest_image.at<cv::Vec3b>(v, u));
      }
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> r(cloud, "r"), g(cloud, "g"), b(cloud, "b");
    for (size_t i = 0; i < points.size(); ++i, ++x, ++y, ++z, ++r, ++g, ++b) {
      *x = points[i][0]; *y = points[i][1]; *z = points[i][2];
      *b = colors[i][0]; *g = colors[i][1]; *r = colors[i][2];
    }
    cloud_pub->publish(cloud);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "RGB-D sampled_points=%zu", points.size());
  }
private:
  cv::Mat latest_image, latest_depth;
  std::array<double, 9> K{};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub, depth_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraProcessor>());
  rclcpp::shutdown();
}
