#pragma once
#include <spdlog/spdlog.h>

#include <iostream>
#include <string>

#include "Spinnaker.h"

using namespace Spinnaker;

namespace sensor {

class SpinManager {
 public:
  SpinManager() {
    system = System::GetInstance();
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    spdlog::info("Spinnaker library version: {}.{}.{}",
                 spinnakerLibraryVersion.major, spinnakerLibraryVersion.minor,
                 spinnakerLibraryVersion.type);

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();
    spdlog::info("Number of cameras detected: {}", numCameras);
    for (int i = 0; i < numCameras; i++) {
      auto pCam = camList.GetByIndex(i);
      pCams.push_back(pCam);
      pCam->Init();
      std::string serial = std::string(pCam->DeviceSerialNumber.ToString());
      serial2idx[serial] = i;
      pCam->DeInit();
    }
    camList.Clear();
  }

  ~SpinManager() { release(); }

  CameraPtr getCamera(int idx) {
    CameraPtr tmpCam = pCams.at(idx);
    return tmpCam;
  }
  CameraPtr getCamera(std::string serial) {
    int idx = serial2idx[serial];
    return getCamera(idx);
  }

  int size() { return int(pCams.size()); }
  std::map<std::string, int> serial2idx;

 private:
  void release() {
    for (int i = 0; i < pCams.size(); i++) {
      pCams[i] = nullptr;
    }
    system->ReleaseInstance();
  }

  SystemPtr system;
  std::vector<CameraPtr> pCams;
};
}  // namespace sensor