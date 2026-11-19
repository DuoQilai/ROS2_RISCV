#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "orb_feature_extractor/orb.hpp"

class FeatureNode : public rclcpp::Node {
public:
  FeatureNode() : Node("orb_feature_extractor") {
    output = create_publisher<sensor_msgs::msg::Image>("/orb/matches", 1);
    input = create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", rclcpp::QoS(10),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        const auto image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        std::vector<cv::KeyPoint> points;
        cv::Mat descriptors;
        extractor.extract(image, points, descriptors);
        if (!previous.empty()) {
          const auto matches = extractor.match_features(old_descriptors, descriptors);
          const auto view = extractor.visualize_matches(previous, image, old_points, points, matches);
          output->publish(*cv_bridge::CvImage(msg->header, "bgr8", view).toImageMsg());
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "ORB keypoints=%zu matches=%zu ratio=0.75",
            points.size(), matches.size());
        }
        previous = image;
        old_points = points;
        old_descriptors = descriptors;
      });
  }
private:
  ORBFeatureExtractor extractor;
  cv::Mat previous, old_descriptors;
  std::vector<cv::KeyPoint> old_points;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr input;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeatureNode>());
  rclcpp::shutdown();
}
