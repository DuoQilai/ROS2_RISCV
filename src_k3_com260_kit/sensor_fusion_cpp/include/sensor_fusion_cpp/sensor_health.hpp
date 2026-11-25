#pragma once
#include <map>
#include <string>
#include <iostream>

class SensorHealthMonitor {
  struct Sensor { double timeout, last=0; bool healthy=true; std::string fallback; };
  std::map<std::string,Sensor> sensors{
    {"lidar",{0.5,0,true,"切换到视觉+IMU紧耦合模式"}},
    {"camera",{1.0,0,true,"切换到激光+IMU模式，禁用视觉回环"}},
    {"imu",{0.2,0,true,"切换到轮式里程计+激光模式，降低动态性能"}},
    {"gps",{2.0,0,true,"切换到纯SLAM定位模式"}}};
public:
  void received(const std::string & name,double stamp) {
    auto & s=sensors.at(name);
    if(!s.healthy) {std::cout<<"恢复: "<<name<<" 已恢复\n";}
    s.last=stamp; s.healthy=true;
  }
  bool check_health(const std::string & name,double stamp) {
    auto & s=sensors.at(name);
    if(stamp-s.last>s.timeout && s.healthy) {
      s.healthy=false;
      std::cout<<"警告: "<<name<<" 故障 (超时"<<stamp-s.last<<"s)\n"
               <<"故障处理: "<<s.fallback<<'\n';
    }
    return s.healthy;
  }
};
