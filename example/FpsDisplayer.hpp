#pragma once
#include <spdlog/spdlog.h>
#include <chrono>
class FpsDisplayer {
  /**
  usage:
  // get fps
  FpsDisplayer fps_displayer("something function", 50);
  fps_displayer.start();
  while(1){
  // something
  // display fps
  fps_displayer.addCount();
  }
  */

 public:
  FpsDisplayer(std::string function_name_, int fps_count_ = 50)
      : FPS_COUNT(fps_count_), function_name(function_name_) {
    count_fps = 0;
  }
  void start() {
    t_meter_fps_start = std::chrono::high_resolution_clock::now();
  }
  void addCount() {
    // get fps
    ++count_fps;
    if (count_fps == FPS_COUNT) {
      auto t_meter_fps_end = std::chrono::high_resolution_clock::now();
      auto t_meter_fps_count =
          std::chrono::duration_cast<std::chrono::microseconds>(
              t_meter_fps_end - t_meter_fps_start)
              .count();
      double mill_sec =
          (static_cast<double>(t_meter_fps_count) / FPS_COUNT) / 1000.;
      double fps = 1000. / mill_sec;
      spdlog::info("{}\t{:.2f} fps\t{:.2f} ms.", function_name, fps, mill_sec);
      count_fps = 0;
      t_meter_fps_start = std::chrono::high_resolution_clock::now();
    }
  }

 private:
  int FPS_COUNT;
  int count_fps;
  std::chrono::high_resolution_clock::time_point t_meter_fps_start;
  std::string function_name;
};
