#pragma once
#include <memory>
#include <string>
#include <vector>

#include "sensors/LRFSensor.hpp"

namespace rp {
namespace standalone {
namespace rplidar {
class RPlidarDriver;
}
}  // namespace standalone
}  // namespace rp

namespace sensor {
class RplidarLRF : public LRFSensor {
 public:
  RplidarLRF(const std::string &com_port);
  ~RplidarLRF();

  int grab(void);
  bool retrieve(std::vector<LRFPoint> &LRF_data);

 private:
  rp::standalone::rplidar::RPlidarDriver *lidar;
  std::vector<float> measured_dists;
  std::vector<float> measured_angles;
};
}  // namespace sensor