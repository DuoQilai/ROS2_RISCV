#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cartographer_ros_msgs/msg/submap_list.hpp>
class CartographerMapQuality : public rclcpp::Node {
public:
  CartographerMapQuality() : Node("cartographer_map_quality") {
    map_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",10,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        ++map_count_; if (msg->data.empty()) { return; }
        const auto mapped=std::count_if(msg->data.begin(),msg->data.end(),[](int v){return v>=0;});
        const auto occupied=std::count(msg->data.begin(),msg->data.end(),100);
        const auto free=std::count(msg->data.begin(),msg->data.end(),0);
        const double coverage=100.0*mapped/msg->data.size();
        RCLCPP_INFO(get_logger(),"建图进度: %.1f%% (占据=%.1f%%, 空闲=%.1f%%)",coverage,
          mapped ? 100.0*occupied/mapped : 0.0,mapped ? 100.0*free/mapped : 0.0);
        if (coverage>90 && map_count_>100) { RCLCPP_INFO(get_logger(),"建图覆盖率达标，建议完成建图并保存地图"); }
      });
    submaps_=create_subscription<cartographer_ros_msgs::msg::SubmapList>("/submap_list",10,
      [this](cartographer_ros_msgs::msg::SubmapList::ConstSharedPtr msg) {
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"子图数量: %zu",msg->submap.size());
      });
  }
private:
  std::size_t map_count_=0;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_;
  rclcpp::Subscription<cartographer_ros_msgs::msg::SubmapList>::SharedPtr submaps_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<CartographerMapQuality>()); rclcpp::shutdown();
}
