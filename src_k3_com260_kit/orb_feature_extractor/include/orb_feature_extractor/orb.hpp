#pragma once
#include <opencv2/features2d.hpp>

class ORBFeatureExtractor {
public:
  ORBFeatureExtractor(int features = 1000, float scale = 1.2f, int levels = 8)
  : orb(cv::ORB::create(features, scale, levels)) {}
  void extract(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors) {
    orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
  }
  std::vector<cv::DMatch> match_features(const cv::Mat & first, const cv::Mat & second, float ratio = 0.75f) {
    std::vector<cv::DMatch> good;
    if (first.empty() || second.rows < 2) { return good; }
    std::vector<std::vector<cv::DMatch>> pairs;
    cv::BFMatcher(cv::NORM_HAMMING).knnMatch(first, second, pairs, 2);
    for (const auto & pair : pairs) {
      if (pair.size() == 2 && pair[0].distance < ratio * pair[1].distance) { good.push_back(pair[0]); }
    }
    return good;
  }
  cv::Mat visualize_matches(const cv::Mat & first, const cv::Mat & second,
      const std::vector<cv::KeyPoint> & kp1, const std::vector<cv::KeyPoint> & kp2,
      const std::vector<cv::DMatch> & matches) {
    cv::Mat result;
    cv::drawMatches(first, kp1, second, kp2, matches, result, cv::Scalar::all(-1),
      cv::Scalar::all(-1), {}, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    return result;
  }
private:
  cv::Ptr<cv::ORB> orb;
};
