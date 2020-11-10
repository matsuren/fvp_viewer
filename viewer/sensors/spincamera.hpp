#pragma once
#include <opencv2/core.hpp>

#include "Spinnaker.h"

using namespace Spinnaker;

class SpinCam;
using SpinCamPtr = std::shared_ptr<SpinCam>;

///////////////////////////////////////////////////////
// SpinCam
///////////////////////////////////////////////////////
class SpinCam {
 public:
  SpinCam(CameraPtr pCam_);

  ~SpinCam();

  bool read(cv::Mat& img_, bool cvt_color = true);
  void grab();
  bool retrieve(cv::Mat& img_, bool cvt_color = true);

  void setFrameRate(double fps);
  void setROI(int offset_x, int offset_y, int width, int height);
  void release();
  void setSoftwareTrigger();
  void displaySetting();
  void setFrameRateAuto(bool flag);
  std::string serial;  // serial number
  std::string model;   // camera model number

  void setWhiteBalanceRatio(double val, std::string select = "Red");

 private:
  CameraPtr pCam;
  bool isGrab = false;
  bool isSoftwareTrigger = false;
  bool isCapturing = false;
  ImagePtr pConverted;  // save in memory
};

///////////////////////////////////////////////////////
// SpinMultiCam
///////////////////////////////////////////////////////
class SpinMultiCam {
 public:
  void addCamera(CameraPtr& cam);

  bool read(std::vector<cv::Mat>& imgs);

  void grab();
  bool retrieve(std::vector<cv::Mat>& imgs);

  void release();

 private:
  std::vector<SpinCamPtr> spincams;
};
