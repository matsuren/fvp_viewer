#pragma once
#include <memory>
#include <iostream>
#include <mutex>
#include <opencv2/core.hpp>
#include "sensors/base_lrf.hpp"
#include "sensors/spincamera.hpp"
#include "sensors/spinmanager.hpp"

namespace fvp {
class System;
class Config;
}  // namespace fvp

class SensorManager {
 public:
  SensorManager(const std::shared_ptr<fvp::Config> &config);
  //-----------------------------------------------------------------------------
  void setFVPSystem(std::shared_ptr<fvp::System> system);
  //-----------------------------------------------------------------------------
  void startCapture(bool useImshow);
  //-----------------------------------------------------------------------------
  void join();
  //-----------------------------------------------------------------------------
  const float LRF_wall_height = 3.0f;

 private:
  // setting parameters
  const std::shared_ptr<fvp::Config> cfg;

  std::unique_ptr<sensor::SpinManager> manager;
  std::shared_ptr<fvp::System> fvp_system;

  // thread
  std::vector<std::thread> ths;
  std::vector<sensor::SpinCamPtr> cams;
  std::vector<cv::Mat> captured_imgs;
  std::vector<std::mutex *> img_mtxs;
  std::shared_ptr<sensor::BaseLRF> LRF_sensor;

  // with viewer
  bool with_viewer = false;

  // -----------------------------------
  int initialize();

  //-----------------------------------------------------------------------------
  void addCamera(const std::string &image_source,
                 std::map<std::string, std::string> &values);

  //-----------------------------------------------------------------------------
  void captureWorker(int cam_id, bool enableFPS);

  //-----------------------------------------------------------------------------
  void viewerWorker();
  //-----------------------------------------------------------------------------
  void LRFWorker(bool enableFPS);


};