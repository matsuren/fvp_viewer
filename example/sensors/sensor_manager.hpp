#pragma once
#include <spdlog/spdlog.h>

#include <array>
#include <fstream>
#include <iostream>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "fvp/fvp_system.hpp"
#include "fvp/config.hpp"
#include "sensors/base_lrf.hpp"
#include "sensors/file_lrf.hpp"
#include "sensors/rplidar_lrf.hpp"
#include "sensors/spincamera.hpp"
#include "sensors/spinmanager.hpp"
#include "sensors/urg_lrf.hpp"
#include "FpsDisplayer.hpp"

// singleton SensorManager CLASS
class SensorManager {
 public:
  // private constractor
  SensorManager(const std::shared_ptr<fvp::Config> &config) : cfg(config) {
    spdlog::info("initialize SensorManager : ");
    manager = std::make_unique<sensor::SpinManager>();
    initialize();
  }
  void setFVPSystem(std::shared_ptr<fvp::System> system) {
    fvp_system = system;
  }
  //-----------------------------------------------------------------------------
  void join() {
    for (auto &it : ths) {
      it.join();
    }
    ths.clear();
  }

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
  int initialize() {
    //////////////////////////
    // add camera
    //////////////////////////
    std::vector<std::string> image_sources = cfg->image_sources();
    // add camera
    for (size_t i = 0; i < image_sources.size(); ++i) {
      std::map<std::string, std::string> values;
      values["framerate"] = std::to_string(cfg->capture_framerate());
      addCamera(image_sources[i], values);
    }

    // Add LRF
    try {
      // const auto lrf_type = sensor::LRFSensorType::FILE;
      const auto lrf_type = sensor::LRFSensorType::RPLIDAR;
      // const auto lrf_type = sensor::LRFSensorType::URG;
      switch (lrf_type) {
        case sensor::LRFSensorType::FILE: {
          std::string str = cfg->LRF_com_port();  //"rp_xy_";
          LRF_sensor = std::make_shared<sensor::FileLRF>(str);
        } break;
        case sensor::LRFSensorType::URG: {
          std::string str = cfg->LRF_com_port();
          char *writable = new char[str.size() + 1];
          std::copy(str.begin(), str.end(), writable);
          writable[str.size()] = '\0';  // don't forget the terminating 0
          char *argv[] = {"exe", "-s", writable};
          LRF_sensor = std::make_shared<sensor::UrgLRF>(3, argv);
          // don't forget to free the string after finished using it
          delete[] writable;
        } break;
        case sensor::LRFSensorType::RPLIDAR: {
          std::string str = cfg->LRF_com_port();
          LRF_sensor = std::make_shared<sensor::RplidarLRF>(str);
        } break;
        default:
          break;
      }
    } catch (...) {
      spdlog::warn("No LRF sensor is detected.");
    }

    return 0;
  }

  //-----------------------------------------------------------------------------
  void addCamera(const std::string &image_source,
                 std::map<std::string, std::string> &values) {
    sensor::SpinCamPtr cam;

    spdlog::info("Adding Spinnaker camera:{}", image_source);
    // Spinnaker Camera (serial number)
    if (image_source.size() == 8) {
      if (manager->serial2idx.find(image_source) != manager->serial2idx.end())
        cam =
            std::make_shared<sensor::SpinCam>(manager->getCamera(image_source));
    }
    // Spinnaker Camera
    else {
      int camera_num = std::stoi(image_source);
      if (camera_num <= manager->size())
        cam = std::make_shared<sensor::SpinCam>(manager->getCamera(camera_num));
    }
    // set fps : Display freshrate
    if (values.find("framerate") != values.end()) {
      double fps = std::stod(values["framerate"]);
      if (cam) {
        cam->setFrameRate(fps);
      }
    }

    if (!cam) {
      spdlog::warn("Cannot find camera:{}", image_source);
    }
    cams.push_back(cam);
    captured_imgs.push_back(cv::Mat());
    img_mtxs.push_back(new std::mutex());
    //// White balance
    // cams.back()->setWhiteBalanceRatio(1.18, "Red");
    // cams.back()->setWhiteBalanceRatio(1.46, "Blue");
  }

  //-----------------------------------------------------------------------------
  void captureWorker(int cam_id, bool enableFPS) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // get fps
    FpsDisplayer fps_displayer("Camera " + std::to_string(cam_id), 160);
    if (enableFPS) fps_displayer.start();
    {
      // save initial image for calibration
      cv::Mat tmp;
      if (cams[cam_id]->read(tmp, false)) {
        {
          cv::cvtColor(tmp, tmp, cv::COLOR_BayerGR2BGR);
          cv::imwrite("img" + std::to_string(cam_id) + ".jpg", tmp);
        }
      }
    }

    try {
      // loop
      while (!fvp_system->checkExit()) {
        cv::Mat tmp;
        if (cams[cam_id]->read(tmp, false)) {
          {
            std::lock_guard<std::mutex> lock(*img_mtxs[cam_id]);
            cv::cvtColor(tmp, captured_imgs[cam_id], cv::COLOR_BayerGR2BGR);
          }

          // Update image for fvp
          int ret = fvp_system->updateImages(captured_imgs[cam_id], cam_id);
          if (ret) {
            // FVP system is not initialized yet
            // Wait for a little
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
          }

        } else {
          if (captured_imgs[cam_id].empty()) {
            // use image
            spdlog::warn("Camera {} is not working.", cam_id);
            spdlog::warn("Use image instead.");
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            break;
          }
        }
        // display fps
        if (enableFPS) fps_displayer.addCount();
      }
    } catch (const std::exception &e) {
      std::cout << e.what();
      fvp_system->checkExit();
    }
  }

  //-----------------------------------------------------------------------------
  void viewerWorker() {
    with_viewer = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // get fps
    FpsDisplayer fps_displayer("viewer", 250);
    fps_displayer.start();
    while (!fvp_system->checkExit()) {
      size_t CAMERA_NUM = cams.size();
      for (size_t i = 0; i < CAMERA_NUM; ++i) {
        if (!captured_imgs[i].empty()) {
          std::lock_guard<std::mutex> lock(*img_mtxs[i]);
          cv::namedWindow(std::to_string(i), cv::WINDOW_NORMAL);
          cv::imshow(std::to_string(i), captured_imgs[i]);
        }
        int key = cv::waitKey(50);
        if (key == 27) fvp_system->threadExit();
      }
      // display fps
      fps_displayer.addCount();
    }
  }
  //-----------------------------------------------------------------------------
  void LRFWorker(bool enableFPS) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::vector<sensor::LRFPoint> LRF_data;

    // get fps
    FpsDisplayer fps_displayer("LRF data ", 160);
    if (enableFPS) fps_displayer.start();

    {
      // save initial scan for calibration
      if (LRF_sensor->grab()) {
        LRF_sensor->retrieve(LRF_data);
        std::ofstream ofs("urg_xy.csv");
        for (const auto &it : LRF_data) {
          ofs << it.x << ", " << it.y << std::endl;
        }
      }
    }

    // loop
    std::vector<float> vertices;
    std::vector<GLuint> elements;
    while (!fvp_system->checkExit()) {
      if (LRF_sensor->grab()) {
        LRF_sensor->retrieve(LRF_data);
        sensor::BaseLRF::getLRFGLdata(LRF_data, vertices, elements,
                                        LRF_wall_height);
        int ret = fvp_system->updateMesh(vertices, elements);
        if (ret) {
          // FVP system is not initialized yet
          // Wait for a little
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      } else {
        spdlog::warn("LRF {} is not working.", cfg->LRF_com_port());
        spdlog::warn("Use saved data instead.");
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
        break;
      }
      // display fps
      if (enableFPS) fps_displayer.addCount();
    }
  }

 public:
  //-----------------------------------------------------------------------------
  void startCapture(bool useImshow) {
    size_t CAMERA_NUM = cams.size();
    //
    bool fps_flag = true;
    for (size_t i = 0; i < CAMERA_NUM; ++i) {
      // display fps only one
      if (cams[i]) {
        ths.push_back(
            std::thread(&SensorManager::captureWorker, this, i, fps_flag));
        fps_flag = false;
      }
    }
    if (useImshow)
      ths.push_back(std::thread(&SensorManager::viewerWorker, this));

    // LRF
    if (LRF_sensor)
      ths.push_back(std::thread(&SensorManager::LRFWorker, this, true));
  }
};