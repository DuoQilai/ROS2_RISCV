#pragma once
#include <cmath>
#include <limits>
#include <queue>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace exploration {
struct Point { double x, y; };
inline double distance(Point a, Point b) { return std::hypot(a.x-b.x, a.y-b.y); }
inline Point world(const nav_msgs::msg::OccupancyGrid & map, cv::Point p) {
  return {map.info.origin.position.x+(p.x+0.5)*map.info.resolution,
          map.info.origin.position.y+(p.y+0.5)*map.info.resolution};
}

// Search only the live SLAM map; the assessment mask is deliberately not an input.
inline bool next_frontier(const nav_msgs::msg::OccupancyGrid & map, Point robot,
                          const std::vector<Point> & visited, Point & target) {
  const int w=map.info.width, h=map.info.height;
  const double r=map.info.resolution;
  cv::Mat free(h,w,CV_8U), occupied(h,w,CV_8U), unknown(h,w,CV_8U);
  for (int i=0;i<w*h;++i) {
    const int value=map.data[i];
    free.data[i]=value>=0 && value<=25 ? 255:0;
    occupied.data[i]=value>=50 ? 255:0;
    unknown.data[i]=value<0 ? 255:0;
  }
  const int radius=std::ceil(0.35/r);
  cv::dilate(occupied,occupied,cv::getStructuringElement(cv::MORPH_ELLIPSE,{2*radius+1,2*radius+1}));
  free.setTo(0,occupied);
  cv::Point seed(-1,-1);
  double closest=1.0;
  for (int y=0;y<h;++y) for (int x=0;x<w;++x) {
    if (!free.at<uchar>(y,x)) continue;
    const double d=distance(world(map,{x,y}),robot);
    if (d<closest) { closest=d; seed={x,y}; }
  }
  if (seed.x<0) return false;
  cv::Mat cost(h,w,CV_32S,cv::Scalar(-1));
  std::queue<cv::Point> queue;
  queue.push(seed); cost.at<int>(seed)=0;
  const cv::Point steps[]={{1,0},{-1,0},{0,1},{0,-1}};
  while (!queue.empty()) {
    const auto p=queue.front(); queue.pop();
    for (auto step:steps) {
      const auto q=p+step;
      if (q.x<0 || q.y<0 || q.x>=w || q.y>=h || !free.at<uchar>(q) || cost.at<int>(q)>=0) continue;
      cost.at<int>(q)=cost.at<int>(p)+1; queue.push(q);
    }
  }
  cv::Mat border, labels, stats, centers;
  cv::dilate(unknown,border,cv::Mat::ones(3,3,CV_8U));
  cv::bitwise_and(border,free,border);
  border.setTo(0,cost<0);
  const int count=cv::connectedComponentsWithStats(border,labels,stats,centers,8);
  double best=std::numeric_limits<double>::infinity();
  for (int label=1;label<count;++label) {
    if (stats.at<int>(label,cv::CC_STAT_AREA)*r<0.4) continue;
    for (int y=0;y<h;++y) for (int x=0;x<w;++x) {
      if (labels.at<int>(y,x)!=label) continue;
      const auto p=world(map,{x,y});
      if (distance(p,robot)<0.8) continue;
      bool tried=false;
      for (auto old:visited) if (distance(p,old)<1.0) { tried=true; break; }
      if (tried) continue;
      const double score=cost.at<int>(y,x)*r/std::min(8.0,std::sqrt(stats.at<int>(label,cv::CC_STAT_AREA)*r));
      if (score<best) { best=score; target=p; }
    }
  }
  return std::isfinite(best);
}
}
