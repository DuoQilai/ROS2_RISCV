#include <iostream>
#include <cassert>
#include <opencv2/imgproc.hpp>
#include "orb_feature_extractor/orb.hpp"
int main() {
  ORBFeatureExtractor extractor;
  cv::Mat image(240, 320, CV_8U);
  cv::RNG random(42);
  random.fill(image, cv::RNG::UNIFORM, 0, 256);
  std::vector<cv::KeyPoint> points;
  cv::Mat descriptors;
  extractor.extract(image, points, descriptors);
  assert(points.size() > 100);
  const auto matches = extractor.match_features(descriptors, descriptors);
  assert(matches.size() > 100);
  for (const auto & match : matches) { assert(match.distance == 0 && match.queryIdx == match.trainIdx); }
  assert(extractor.match_features({}, {}).empty());
  assert(extractor.match_features(descriptors, descriptors.row(0)).empty());
  std::cout << "ORB known image: keypoints=" << points.size() << " identity_matches=" << matches.size() << '\n';
}
