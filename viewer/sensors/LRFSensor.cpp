#include "LRFSensor.hpp"

#include <iostream>
#include <spdlog/spdlog.h>

#include "urg/Connection_information.hpp"
#include "urg/Urg_driver.h"

using namespace qrk;
//-----------------------------------------------------------------------------
LRFSensor::LRFSensor(int argc, char *argv[]) {
  Connection_information information(argc, argv);

  // Connects to the sensor
  if (!urg.open(information.device_or_ip_name(),
                information.baudrate_or_port_number(),
                information.connection_type())) {
    spdlog::warn("Urg_driver::open():{}:{}", information.device_or_ip_name(),
                 urg.what());
    isOpened = false;
  } else {
    isOpened = true;
    urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);

    // initialize
    long time_stamp = 0;

    // get distance
    if (!urg.get_distance(data, &time_stamp)) {
      spdlog::warn("Urg_driver::get_distance(): {}", urg.what());
    }
    size_t data_n = data.size();

    index2cos.clear();
    index2sin.clear();
    for (size_t i = 0; i < data_n; ++i) {
      double radian = urg.index2rad(int(i));
      index2cos.push_back(cos(radian));
      index2sin.push_back(sin(radian));
    }
  }
}
//-----------------------------------------------------------------------------
int LRFSensor::grab() {
  if (!isOpened) return 0;

  long time_stamp = 0;

  // get distance
  if (!urg.get_distance(data, &time_stamp)) {
    spdlog::warn("Urg_driver::get_distance(): {}", urg.what());
    return 0;
  }
  return 1;
}

//-----------------------------------------------------------------------------
bool LRFSensor::retrieve(std::vector<LRFPoint> &LRF_data) {
  LRF_data.clear();
  LRF_data.reserve(1200);
  // Prints the X-Y coordinates for all the measurement points
  long min_distance = urg.min_distance();
  long max_distance = urg.max_distance();
  size_t data_n = data.size();

  for (size_t i = 0; i < data_n; ++i) {
    long l = data[i];
    if ((l <= min_distance) || (l >= max_distance)) {
      continue;
    }

    /* double radian = urg.index2rad(int(i));
    float x = float(l * cos(radian));
    float y = float(l * sin(radian)); */
    float x = float(l * index2cos[i]);
    float y = float(l * index2sin[i]);
    // coordinate conversion x -> -y, y -> x
    LRFPoint tmp_pair(-y / 1000.0, x / 1000.0);
    LRF_data.push_back(tmp_pair);
  }
  return true;
}
//-----------------------------------------------------------------------------
