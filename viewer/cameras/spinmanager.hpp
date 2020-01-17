#pragma once
#include <iostream>
#include <string>

#include "Spinnaker.h"

using namespace Spinnaker;

class SpinManager {
public:
  SpinManager() {
    system = System::GetInstance();
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    std::cout << "Spinnaker library version: " << spinnakerLibraryVersion.major
              << "." << spinnakerLibraryVersion.minor << "."
              << spinnakerLibraryVersion.type << "."
              << spinnakerLibraryVersion.build << std::endl
              << std::endl;

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();
    std::cout << "Number of cameras detected: " << numCameras << std::endl
              << std::endl;
    for (size_t i = 0; i < numCameras; i++) {
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

  int size() { return pCams.size(); }
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
