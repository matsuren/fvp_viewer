
#include "sensors/file_lrf.hpp"

#include <spdlog/spdlog.h>
#include <fstream>
#include <iostream>

namespace sensor {

//-----------------------------------------------------------------------------
FileLRF::FileLRF(const std::string base_filename) : base_fname(base_filename), cnt(0) {
  isOpened = true;
}
//-----------------------------------------------------------------------------
int FileLRF::grab() {
  if (!isOpened) return 0;

  grab_fname = base_fname + std::to_string(cnt) + ".csv";
  cnt++;
  std::ifstream ifs_lrf(grab_fname);
  if (!ifs_lrf) {
    spdlog::warn("Cannot load LRF data file:{}", grab_fname);
    isOpened = false;
    return false;
  }
  return 1;
}

//-----------------------------------------------------------------------------
bool FileLRF::retrieve(std::vector<LRFPoint> &LRF_data) {
  LRF_data.clear();
  LRF_data.reserve(1600);

  spdlog::info("Loading {}", grab_fname);
  std::ifstream ifs_lrf(grab_fname);
  // read from csv file
  LRF_data.clear();
  std::string tmp_str;
  while (std::getline(ifs_lrf, tmp_str)) {
    std::vector<std::string> ret_str = LRFSensor::split(tmp_str, ",");
    LRFPoint tmp_pair(std::stof(ret_str[0]), std::stof(ret_str[1]));
    LRF_data.push_back(tmp_pair);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  return true;
}
}  // namespace sensor
