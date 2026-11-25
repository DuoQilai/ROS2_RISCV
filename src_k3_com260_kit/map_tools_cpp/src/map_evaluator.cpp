#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char ** argv) {
  if (argc < 2 || argc > 3) { std::cerr << "Usage: map_evaluator MAP.pgm [GROUND_TRUTH.pgm]\n"; return 2; }
  const auto image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  if (image.empty()) { std::cerr << "Cannot read map image\n"; return 1; }
  cv::Mat edges;
  cv::Canny(image, edges, 50, 150);
  const double cells = image.total();
  std::cout << "known_grid_ratio=" << 100.0*cv::countNonZero(image != 205)/cells
            << "% clarity=" << cv::countNonZero(edges)/cells << " total_cells=" << image.total();
  if (argc == 3) {
    const auto truth = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    if (truth.empty() || truth.size() != image.size()) { std::cerr << "\nGround truth must have the same size and coordinates\n"; return 1; }
    cv::Mat difference;
    cv::absdiff(image, truth, difference);
    std::cout << " accuracy=" << 1.0-cv::countNonZero(difference > 50)/cells;
  } else { std::cout << " accuracy=unmeasured"; }
  std::cout << '\n';
}
