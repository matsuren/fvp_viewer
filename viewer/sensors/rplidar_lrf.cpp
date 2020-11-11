#define _USE_MATH_DEFINES
#include "sensors/rplidar_lrf.hpp"

#include <math.h>
#include <rplidar.h>
#include <spdlog/spdlog.h>

#include <iostream>
#include <memory>

namespace sensor {
//-----------------------------------------------------------------------------
RplidarLRF::RplidarLRF(const std::string &com_port) {
  using namespace rp::standalone::rplidar;
  spdlog::info("Connecting to RplidarLRF");
  lidar = RPlidarDriver::CreateDriver();
  if (!lidar) {
    spdlog::warn("Cannot create RPlidarDriver");
    throw std::runtime_error("Rplidar runtime error");
  }

  // 256000 for A3. 115200 for else?
  const std::string com_port_cmd = "\\\\.\\" + std::string(com_port);
  u_result res = lidar->connect(com_port_cmd.c_str(), 256000);
  if (IS_FAIL(res)) {
    spdlog::warn("Cannot connect to Rplidar {}", com_port);
    throw std::runtime_error("Rplidar runtime error");
  }

  rplidar_response_device_info_t devinfo;
  res = lidar->getDeviceInfo(devinfo);
  if (IS_FAIL(res)) {
    spdlog::warn("Cannot get device infomation");
    throw std::runtime_error("Rplidar runtime error");
  }
  // spdlog::info("Firmware Ver: {}.{}", devinfo.firmware_version >> 8,
  //             devinfo.firmware_version & 0xFF);
  // spdlog::info("Hardware Rev: {}", (int)devinfo.hardware_version);

  // Start scan
  lidar->startMotor();
  std::vector<RplidarScanMode> scanModes;
  lidar->getAllSupportedScanModes(scanModes);

  // std::string request_mode = "Standard";
  //std::string request_mode = "Express";
  // std::string request_mode = "Boost";
  // std::string request_mode = "Sensitivity";
   std::string request_mode = "Stability";

  RplidarScanMode mode = scanModes[0];
  for (auto &it : scanModes) {
    if (it.scan_mode == request_mode) {
      mode = it;
    }
  }
  lidar->startScanExpress(false, mode.id);
  isOpened = true;
}

RplidarLRF::~RplidarLRF() {
  using namespace rp::standalone::rplidar;
  lidar->stopMotor();
  lidar->disconnect();
  RPlidarDriver::DisposeDriver(lidar);
}
//-----------------------------------------------------------------------------
int RplidarLRF::grab() {
  if (!isOpened) return false;

  measured_angles.clear();
  measured_dists.clear();
  measured_angles.reserve(8192);
  measured_dists.reserve(8192);
  rplidar_response_measurement_node_hq_t nodes[8192];
  size_t nodeCount =
      sizeof(nodes) / sizeof(rplidar_response_measurement_node_hq_t);

  u_result res = lidar->grabScanDataHq(nodes, nodeCount);

  spdlog::debug("Rplidar data count: {}", nodeCount);

  if (IS_OK(res)) {
    lidar->ascendScanData(nodes, nodeCount);
    for (int pos = 0; pos < (int)nodeCount; ++pos) {
      if (nodes[pos].quality == 0) continue;

      // If too short, then skip
      float distance_in_meters = nodes[pos].dist_mm_q2 / 1000.f / (1 << 2);
      if (distance_in_meters < 0.1) continue;
      float angle_in_degrees = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
      float angle_in_rad = angle_in_degrees / 180.0 * M_PI;

      measured_angles.push_back(angle_in_rad);
      measured_dists.push_back(distance_in_meters);

      // printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
      //       (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "
      //       ", (nodes[pos].angle_z_q14 * 90.f / (1 << 14)),
      //       nodes[pos].dist_mm_q2 / 4.0f, nodes[pos].quality);
    }
  } else {
    spdlog::warn("Cannot grab ScanDataHq");
    isOpened = false;
    throw std::runtime_error("Rplidar runtime error");
  }
  return true;
}

//-----------------------------------------------------------------------------
bool RplidarLRF::retrieve(std::vector<LRFPoint> &LRF_data) {
  LRF_data.clear();
  LRF_data.reserve(2000);

  if (measured_angles.size() != measured_dists.size())
    throw std::runtime_error("Should be the same size");
  int num_total = int(measured_angles.size());
  for (size_t i = 0; i < num_total; i++) {
    int idx = num_total - i - 1;
    float x = measured_dists[idx] * sin(measured_angles[idx]);
    float y = measured_dists[idx] * cos(measured_angles[idx]);
    LRF_data.emplace_back(x, y);
  }
  return true;
}
}  // namespace sensor