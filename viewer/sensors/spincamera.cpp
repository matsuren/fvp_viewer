
#include "spincamera.hpp"

#include <spdlog/spdlog.h>

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <string>

#include "Spinnaker.h"

using namespace Spinnaker;

namespace sensor {

class SpinCam;
using SpinCamPtr = std::shared_ptr<SpinCam>;

///////////////////////////////////////////////////////
// SpinCam
///////////////////////////////////////////////////////
SpinCam::SpinCam(CameraPtr pCam_) {
  try {
    pCam = pCam_;
    pCam->Init();
    pCam->PixelFormat.SetValue(PixelFormat_BayerGB8);
    // Image acquisition
    pCam->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
    // Newest frame only to reduce latency
    pCam->TLStream.StreamBufferHandlingMode.SetValue(
        StreamBufferHandlingMode_NewestOnly);

    // camera info
    serial = pCam->DeviceSerialNumber.ToString();
    model = pCam->DeviceModelName.ToString();
    spdlog::info("Pixel format:{}", pCam->PixelFormat.ToString());
    spdlog::info("Device model:{}", model);
    spdlog::info("Serial number:{}", serial);
    if (model == "Grasshopper3 GS3-U3-41C6C") {
      // spdlog::info("Set ROI since Model is {}", model);
      // setROI(224, 224, 1600, 1600);
    }

    // White balance
    setWhiteBalanceRatio(1.18, "Red");
    setWhiteBalanceRatio(1.46, "Blue");

  } catch (Spinnaker::Exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
  }
}

SpinCam::~SpinCam() {
  if (pCam) release();
}

bool SpinCam::read(cv::Mat& img_, bool cvt_color) {
  grab();
  bool ret = retrieve(img_, cvt_color);
  return ret;
}

void SpinCam::grab() {
  if (!isCapturing) {
    // Start capture
    displaySetting();
    pCam->BeginAcquisition();
    isCapturing = true;
  }

  if (isSoftwareTrigger) pCam->TriggerSoftware();
  isGrab = true;
}

bool SpinCam::retrieve(cv::Mat& img_, bool cvt_color) {
  // empty image first
  img_ = cv::Mat();
  if (!isGrab && isSoftwareTrigger) {
    std::cout << "Software trigger is set. Please grab() first." << std::endl;
    return false;
  }
  ImagePtr pResultImage;
  try {
    if (isSoftwareTrigger) {
      pResultImage = pCam->GetNextImage(30);
    } else {
      pResultImage = pCam->GetNextImage(1000);
    }
  } catch (Spinnaker::Exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
    return false;
  }

  if (pResultImage->IsIncomplete()) {
    return false;
  }

  int width = pResultImage->GetWidth();
  int height = pResultImage->GetHeight();
  cv::Mat img(height, width, CV_8UC1, pResultImage->GetData());
  if (cvt_color) {
    cv::cvtColor(img, img_, cv::COLOR_BayerGR2BGR);
  } else {
    img_ = img.clone();
  }

  pResultImage->Release();
  return true;
}

void SpinCam::setFrameRate(double fps) {
  spdlog::debug("Setup framerate: {}", fps);
  pCam->TriggerMode.SetValue(TriggerMode_Off);
  pCam->TriggerSource.SetValue(TriggerSource_Line0);
  setFrameRateAuto(false);
  pCam->AcquisitionFrameRate.SetValue(fps);
  isSoftwareTrigger = false;
}

void SpinCam::setROI(int offset_x, int offset_y, int width, int height) {
  spdlog::debug("Set ROI:{},{},{},{}", offset_x, offset_y, width, height);
  if (pCam->OffsetX.GetValue() < offset_x) {
    int _width = width / 32;
    pCam->Width.SetValue(_width * 32);
    int _height = height / 2;
    pCam->Height.SetValue(_height * 2);
    pCam->OffsetX.SetValue(offset_x);
    pCam->OffsetY.SetValue(offset_y);
  } else {
    pCam->OffsetX.SetValue(offset_x);
    pCam->OffsetY.SetValue(offset_y);
    int _width = width / 32;
    pCam->Width.SetValue(_width * 32);
    int _height = height / 2;
    pCam->Height.SetValue(_height * 2);
  }
}

void SpinCam::release() {
  if (isCapturing) pCam->EndAcquisition();
  pCam->DeInit();
  pCam = nullptr;
}

void SpinCam::setSoftwareTrigger() {
  spdlog::info("Set softwareTrigger");
  pCam->TriggerMode.SetValue(TriggerMode_Off);
  pCam->TriggerSource.SetValue(TriggerSource_Software);
  pCam->TriggerMode.SetValue(TriggerMode_On);
  isSoftwareTrigger = true;

  // Limit exposure time
  //   pCam->ExposureAuto.SetValue(ExposureAuto_Continuous);
  //   pCam->AutoExposureExposureTimeUpperLimit.SetValue(20000);
  // Fix exposure time
  pCam->ExposureAuto.SetValue(ExposureAuto_Once);
  //  pCam->ExposureAuto.SetValue(ExposureAuto_Off);
  //  pCam->ExposureTime.SetValue(10000);
}

void SpinCam::displaySetting() {
  spdlog::info("#### Current setting ####");
  spdlog::info("Width x Height, (offsetX x offsetY): {} x {}, ({} x {})",
               pCam->Width.ToString(), pCam->Height.ToString(),
               pCam->OffsetX.ToString(), pCam->OffsetY.ToString());

  if (isSoftwareTrigger) {
    spdlog::info("Software trigger");
  } else {
    spdlog::info("FPS: {}", pCam->AcquisitionFrameRate.ToString());
  }
}

void SpinCam::setFrameRateAuto(bool flag) {
  int64_t value;

  // Turning AcquisitionFrameRateAuto on or off
  GenApi::CEnumerationPtr ptrFrameRateAuto =
      pCam->GetNodeMap().GetNode("AcquisitionFrameRateAuto");
  if (!IsAvailable(ptrFrameRateAuto) || !IsWritable(ptrFrameRateAuto)) {
    std::cout << "Unable to set AcquisitionFrameRateAuto..." << std::endl
              << std::endl;
    return;
  }
  if (flag) {
    GenApi::CEnumEntryPtr ptrFrameRateAutoMode =
        ptrFrameRateAuto->GetEntryByName("Continuous");
    if (!IsAvailable(ptrFrameRateAutoMode) ||
        !IsReadable(ptrFrameRateAutoMode)) {
      std::cout << "Unable to set AcquisitionFrameRateAuto to Continuous. "
                   "Aborting..."
                << std::endl
                << std::endl;
      return;
    }
    value = ptrFrameRateAutoMode->GetValue();
  } else {
    GenApi::CEnumEntryPtr ptrFrameRateAutoMode =
        ptrFrameRateAuto->GetEntryByName("Off");
    if (!IsAvailable(ptrFrameRateAutoMode) ||
        !IsReadable(ptrFrameRateAutoMode)) {
      std::cout << "Unable to set AcquisitionFrameRateAuto to OFF. Aborting..."
                << std::endl
                << std::endl;
      return;
    }
    // Set value
    value = ptrFrameRateAutoMode->GetValue();
  }
  ptrFrameRateAuto->SetIntValue(value);
}

void SpinCam::setWhiteBalanceRatio(double val, std::string select) {
  if (select == "Red")
    pCam->BalanceRatioSelector.SetValue(BalanceRatioSelector_Red);
  else if (select == "Blue")
    pCam->BalanceRatioSelector.SetValue(BalanceRatioSelector_Blue);
  pCam->BalanceRatio.SetValue(val);
}

///////////////////////////////////////////////////////
// SpinMultiCam
///////////////////////////////////////////////////////
void SpinMultiCam::addCamera(CameraPtr& cam) {
  // Only software trigger is possible
  SpinCamPtr cam_ptr = std::make_shared<SpinCam>(cam);
  cam_ptr->setSoftwareTrigger();
  spincams.push_back(cam_ptr);
}

bool SpinMultiCam::read(std::vector<cv::Mat>& imgs) {
  grab();
  bool ret = retrieve(imgs);
  return ret;
}

void SpinMultiCam::grab() {
  for (auto& it : spincams) {
    it->grab();
  }
}

bool SpinMultiCam::retrieve(std::vector<cv::Mat>& imgs) {
  imgs.clear();
  bool ret = true;
  for (auto& it : spincams) {
    cv::Mat tmpimg;
    ret &= it->retrieve(tmpimg);
    imgs.push_back(tmpimg);
  }
  return ret;
}

void SpinMultiCam::release() {
  for (auto& it : spincams) {
    it->release();
  }
}
}  // namespace sensor
