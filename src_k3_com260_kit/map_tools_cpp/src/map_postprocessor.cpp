#include <filesystem>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

int main(int argc, char ** argv) {
  if (argc != 4) { std::cerr << "Usage: map_postprocessor INPUT.pgm INPUT.yaml OUTPUT_PREFIX\n"; return 2; }
  try {
    auto image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if (image.empty()) { throw std::runtime_error("Cannot read map image"); }
    auto meta = YAML::LoadFile(argv[2]);
    const cv::Mat observed_obstacles = image < 200;
    cv::medianBlur(image, image, 3);
    cv::morphologyEx(image, image, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5}));
    image.setTo(0, observed_obstacles);
    cv::Mat binary;
    cv::threshold(image, binary, 200, 255, cv::THRESH_BINARY);
    cv::erode(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, {7,7}));
    image.setTo(0, binary == 0);
    std::vector<cv::Point> points;
    cv::findNonZero(image, points);
    if (!points.empty()) {
      const auto bounds = cv::boundingRect(points);
      image = image(bounds).clone();
      const double resolution = meta["resolution"].as<double>();
      meta["origin"][0] = meta["origin"][0].as<double>() + bounds.x*resolution;
      meta["origin"][1] = meta["origin"][1].as<double>() + bounds.y*resolution;
    }
    const std::string prefix = argv[3];
    if (!cv::imwrite(prefix+".pgm", image)) { throw std::runtime_error("Cannot write map image"); }
    meta["image"] = std::filesystem::path(prefix).filename().string()+".pgm";
    meta["free_thresh"] = 0.196;
    std::ofstream output(prefix+".yaml");
    output << meta << '\n';
    if (!output) { throw std::runtime_error("Cannot write map metadata"); }
    std::cout << "地图已保存: " << prefix << ".pgm, " << prefix << ".yaml\n";
  } catch (const std::exception & e) { std::cerr << e.what() << '\n'; return 1; }
}
