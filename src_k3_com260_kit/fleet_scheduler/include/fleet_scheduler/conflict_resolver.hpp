#pragma once
#include <cmath>
#include <map>
#include <string>

class ConflictResolver {
  struct Zone {double x,y,radius;};
  std::map<std::string,Zone> zones;
public:
  bool check_conflict(const std::string & robot,double x,double y) const {
    for(const auto & [other,zone]:zones) {
      if(other!=robot && std::hypot(zone.x-x,zone.y-y)<1.0) {return true;}
    }
    return false;
  }
  void reserve_zone(const std::string & robot,double x,double y,double radius=1.0) {
    zones[robot]={x,y,radius};
  }
  void release_zone(const std::string & robot) {zones.erase(robot);}
};
