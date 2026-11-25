#include <iostream>
#include <stdexcept>
#include "auto_explorer/frontiers.hpp"

int main() {
  nav_msgs::msg::OccupancyGrid map;
  map.info.width=80; map.info.height=60; map.info.resolution=0.1;
  map.data.assign(80*60,100);
  for (int y=5;y<55;++y) for (int x=5;x<75;++x) map.data[y*80+x]=x<40?0:-1;
  exploration::Point goal;
  auto check=[](bool ok,const char * name) {
    if (!ok) throw std::runtime_error(name);
    std::cout<<name<<": PASS\n";
  };
  check(exploration::next_frontier(map,{1,3},{},goal),"reachable frontier");
  check(goal.x>3.8 && goal.x<4.1 && goal.y>0.8 && goal.y<5.2,"known free goal with wall clearance");
  const auto first=goal;
  check(exploration::next_frontier(map,{1,3},{first},goal) && exploration::distance(first,goal)>=1.0,"skip attempted target");
  for (int y=5;y<55;++y) map.data[y*80+25]=100;
  check(!exploration::next_frontier(map,{1,3},{},goal),"do not cross separating wall");
  map.data.assign(80*60,0);
  check(!exploration::next_frontier(map,{1,3},{},goal),"no frontier in completed map");
  map.data.assign(80*60,-1);
  check(!exploration::next_frontier(map,{1,3},{},goal),"no goal without known free space");
}
