#include <cmath>
#include <filesystem>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

int main(int argc,char ** argv) {
  if (argc!=3) { std::cerr<<"Usage: map_coverage MAP.yaml EXPLORATION_AREA.yaml\n"; return 2; }
  try {
    const auto map=YAML::LoadFile(argv[1]), area=YAML::LoadFile(argv[2]);
    auto read=[](const char * path,const YAML::Node & meta) {
      return cv::imread((std::filesystem::path(path).parent_path()/meta["image"].as<std::string>()).string(),cv::IMREAD_GRAYSCALE);
    };
    const auto image=read(argv[1],map), mask=read(argv[2],area);
    if (image.empty() || mask.empty()) throw std::runtime_error("Cannot read map or area mask");
    const double mr=map["resolution"].as<double>(), ar=area["resolution"].as<double>();
    if (mr<=0 || ar<=0) throw std::runtime_error("Invalid map resolution");
    const double yaw=map["origin"][2].as<double>();
    const double cosine=std::cos(yaw), sine=std::sin(yaw);
    int known=0,total=cv::countNonZero(mask);
    for (int row=0;row<mask.rows;++row) for (int col=0;col<mask.cols;++col) {
      if (!mask.at<uchar>(row,col)) continue;
      const double dx=area["origin"][0].as<double>()+(col+0.5)*ar-map["origin"][0].as<double>();
      const double dy=area["origin"][1].as<double>()+(mask.rows-row-0.5)*ar-map["origin"][1].as<double>();
      const int x=std::floor((cosine*dx+sine*dy)/mr), y=std::floor((-sine*dx+cosine*dy)/mr);
      if (x>=0 && y>=0 && x<image.cols && y<image.rows && image.at<uchar>(image.rows-1-y,x)!=205) ++known;
    }
    if (!total) throw std::runtime_error("Empty exploration area");
    const double square_meters=total*ar*ar, coverage=100.0*known/total;
    const bool pass=square_meters>=500.0 && coverage>95.0;
    std::cout<<"area_m2="<<square_meters<<" coverage="<<coverage<<"% known="<<known
             <<" total="<<total<<" pass="<<(pass?"true":"false")<<'\n';
    return pass?0:1;
  } catch (const std::exception & e) { std::cerr<<e.what()<<'\n'; return 1; }
}
