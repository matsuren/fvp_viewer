#pragma once
#include <array>
#include <fstream>
#include <iostream>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "GLDataManager.hpp"
#include "main.hpp"
#include "sensors/LRFSensor.hpp"
#include "sensors/spincamera.hpp"
#include "sensors/spinmanager.hpp"
#include "utils/FpsDisplayer.hpp"
#include "utils/SettingParameters.hpp"

// singleton SensorManager CLASS
class SensorManager {
 public:
  // private constractor
  SensorManager(const std::shared_ptr<fvp::Config> &config) : cfg(config) {
    std::cout << "initialize SensorManager : " << std::endl;
    manager = std::make_unique<SpinManager>();
    initialize();

    startCapture(false);
  }
  void setSensorDataManager(std::shared_ptr<fvp::GLDataManager> &manager) {
    gl_data_mgr = manager;
  }
  //-----------------------------------------------------------------------------
  void join() {
    for (auto &it : ths) {
      it.join();
    }
    ths.clear();
  }

 private:
  // setting parameters
  const std::shared_ptr<fvp::Config> cfg;

  std::unique_ptr<SpinManager> manager;
  std::shared_ptr<fvp::GLDataManager> gl_data_mgr;

  // thread
  std::vector<std::thread> ths;
  std::vector<SpinCamPtr> cams;
  std::vector<cv::Mat> captured_imgs;
  std::vector<std::mutex *> img_mtxs;
  std::unique_ptr<LRFSensor> LRF_sensor;
  std::vector<LRFPoint> LRF_data;

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

    std::string str = cfg->LRF_com_port();
    char *writable = new char[str.size() + 1];
    std::copy(str.begin(), str.end(), writable);
    writable[str.size()] = '\0';  // don't forget the terminating 0
    char *argv[] = {"exe", "-s", writable};
    LRF_sensor = std::make_unique<LRFSensor>(3, argv);
    std::cout << "**** LRF port : " << argv[2] << std::endl;
    // don't forget to free the string after finished using it
    delete[] writable;
    return 0;
  }

  //-----------------------------------------------------------------------------
  void addCamera(const std::string &image_source,
                 std::map<std::string, std::string> &values) {
    SpinCamPtr cam;
    // Spinnaker Camera (serial number)
    if (image_source.size() == 8) {
      if (manager->serial2idx.find(image_source) != manager->serial2idx.end())
        cam = std::make_shared<SpinCam>(manager->getCamera(image_source));
    }
    // Spinnaker Camera
    else {
      int camera_num = std::stoi(image_source);
      if (camera_num <= manager->size())
        cam = std::make_shared<SpinCam>(manager->getCamera(camera_num));
    }
    // set fps : Display freshrate
    if (values.find("framerate") != values.end()) {
      double fps = std::stod(values["framerate"]);
       if(cam)
      	cam->setFrameRate(fps);
    }
    cams.push_back(cam);
    captured_imgs.push_back(cv::Mat());
    img_mtxs.push_back(new std::mutex());
    //// White balance
    // cams.back()->setWhiteBalanceRatio(1.18, "Red");
    // cams.back()->setWhiteBalanceRatio(1.46, "Blue");
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
    ths.push_back(std::thread(&SensorManager::LRFWorker, this, true));
  }

  //-----------------------------------------------------------------------------
  void captureWorker(int cam_id, bool enableFPS) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // get fps
    FpsDisplayer fps_displayer("capture image " + std::to_string(cam_id), 160);
    if (enableFPS) fps_displayer.start();

    std::vector<cv::Mat> capture_image;

    try {
      // loop
      while (!checkExit()) {
        cv::Mat tmp;
        if (cams[cam_id]->read(tmp, false)) {
          {
            std::lock_guard<std::mutex> lock(*img_mtxs[cam_id]);
            cv::cvtColor(tmp, captured_imgs[cam_id], cv::COLOR_BayerGR2BGR);
          }
          gl_data_mgr->updateImgs(captured_imgs[cam_id], cam_id);
          bool ret = true;
          if (!ret) {
            // use image
            std::cout << "\n/*********** Something wrong in captureWorker !!! "
                         "Use image in sample folder. ***********\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            break;
          }
        } else {
          if (captured_imgs[cam_id].empty()) {
            // use image
            std::cout << "\n/*********** camera " << cam_id
                      << " is not working!  use image. ***********\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            break;
          }
        }
        // display fps
        if (enableFPS) fps_displayer.addCount();
      }
    } catch (const std::exception &e) {
      std::cout << e.what();
      threadExit();
    }
  }

  //-----------------------------------------------------------------------------
  void viewerWorker() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // get fps
    FpsDisplayer fps_displayer("viewer", 250);
    fps_displayer.start();
    while (!checkExit()) {
      size_t CAMERA_NUM = cams.size();
      for (size_t i = 0; i < CAMERA_NUM; ++i) {
        if (!captured_imgs[i].empty()) {
          std::lock_guard<std::mutex> lock(*img_mtxs[i]);
          cv::namedWindow(std::to_string(i), cv::WINDOW_NORMAL);
          cv::imshow(std::to_string(i), captured_imgs[i]);
        } 
        int key = cv::waitKey(50);
        if (key == 27) threadExit();
      }
      // display fps
      fps_displayer.addCount();
    }
  }
  //-----------------------------------------------------------------------------
  void LRFWorker(bool enableFPS) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // get fps
    FpsDisplayer fps_displayer("LRF data ", 160);
    if (enableFPS) fps_displayer.start();

    // loop
    while (!checkExit()) {
      if (LRF_sensor->grab()) {
        LRF_sensor->retrieve(LRF_data);
        gl_data_mgr->updateLRF(LRF_data);
      } else {
        // use image
        std::cout << "\n/*********** LRF  is not working!  use saved data. "
                     "***********\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
        break;
      }
      // display fps
      if (enableFPS) fps_displayer.addCount();
    }
  }
};