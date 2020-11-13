#pragma once
#include <vector>
#include <string>
#include "sensors/LRFSensor.hpp"

namespace sensor {
class FileLRF : public LRFSensor {
 public:
  FileLRF(const std::string base_filename);

  int grab(void);
  bool retrieve(std::vector<LRFPoint> &LRF_data);

 private:
  const std::string base_fname;
  std::string grab_fname;
  int cnt;
};
}  // namespace sensor
