#pragma once
#include <algorithm>
#include <string>
#include <stdexcept>

struct SpeedParameters { double linear, angular, acceleration; };

inline SpeedParameters recommend_speed(const std::string & robot, const std::string & environment) {
  SpeedParameters speed;
  if (robot == "differential") { speed = {0.5, 1.0, 2.5}; }
  else if (robot == "ackermann") { speed = {0.8, 0.5, 1.5}; }
  else { throw std::invalid_argument("unknown robot type"); }
  if (environment == "narrow") { speed.linear *= 0.6; speed.acceleration *= 0.7; }
  else if (environment == "open") { speed.linear *= 1.2; }
  return speed;
}

inline double recommend_lookahead(double speed, double frequency, double length) {
  if (frequency <= 0) { throw std::invalid_argument("frequency must be positive"); }
  return std::clamp((speed * 3 * 10 / frequency + length * 2) / 2, 0.3, 3.0);
}
