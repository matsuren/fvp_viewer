#pragma once
#include <urg/Urg_driver.h>

#include <vector>

#include "sensors/LRFSensor.hpp"

namespace sensor {
class UrgLRF : public LRFSensor {
 public:
  UrgLRF(int argc, char *argv[]);

  int grab(void);
  bool retrieve(std::vector<LRFPoint> &LRF_data);

 private:
  qrk::Urg_driver urg;
  std::vector<long> data;

  // double radian = urg.index2rad(int(i));
  // index2cos : cos(radian)
  std::vector<double> index2cos;
  // index2sin : sin(radian)
  std::vector<double> index2sin;
};
}  // namespace sensor
