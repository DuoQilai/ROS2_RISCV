#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
class MapQualityMonitor : public rclcpp::Node {
public:
  MapQualityMonitor() : Node("map_quality_monitor") {
    sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",10,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        if (++count_%10!=0 || msg->data.empty()) { return; }
        const auto free=std::count(msg->data.begin(),msg->data.end(),0);
        const auto occupied=std::count(msg->data.begin(),msg->data.end(),100);
        const auto unknown=std::count_if(msg->data.begin(),msg->data.end(),[](int v){return v<0;});
        const double coverage=100.0*(free+occupied)/msg->data.size();
        RCLCPP_INFO(get_logger(),"建图质量 [#%zu]: 覆盖=%.1f%% (空闲=%td, 占据=%td, 未知=%td) 分辨率=%.3fm",
          count_,coverage,free,occupied,unknown,msg->info.resolution);
        if (coverage>95) { RCLCPP_INFO(get_logger(),"环境已充分探索，建议保存地图!"); }
      });
  }
private:
  std::size_t count_=0; rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<MapQualityMonitor>()); rclcpp::shutdown();
}
