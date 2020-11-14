#pragma once
#include <spdlog/spdlog.h>

#include <fstream>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

namespace sensor {

enum class LRFSensorType { FILE=0, URG, RPLIDAR };

struct LRFPoint {
  float x;
  float y;
  LRFPoint() {
    x = 0.0f;
    y = 0.0f;
  }
  LRFPoint(float x_, float y_) {
    x = x_;
    y = y_;
  }
};

class BaseLRF {
 public:
  BaseLRF(){};

  virtual int grab(void) = 0;
  virtual bool retrieve(std::vector<LRFPoint> &LRF_data) = 0;

  static int loadLRFDataCSV(const std::string lrf_fname,
                            std::vector<LRFPoint> &LRF_data) {
    spdlog::info("Loading {}", lrf_fname);
    std::ifstream ifs_lrf(lrf_fname);
    if (!ifs_lrf) {
      spdlog::error("Cannot load LRF data file:{}", lrf_fname);
      throw std::runtime_error("No LRF data file");
    }
    // read from csv file
    LRF_data.clear();
    std::string str;
    while (std::getline(ifs_lrf, str)) {
      std::vector<std::string> ret_str = BaseLRF::split(str, ",");
      LRFPoint tmp_pair(std::stof(ret_str[0]), std::stof(ret_str[1]));
      LRF_data.push_back(tmp_pair);
    }
    return 0;
  }

  static std::vector<std::string> split(const std::string &s,
                                        const std::string delim) {
    std::vector<std::string> result;
    result.clear();

    using string = std::string;
    string::size_type pos = 0;

    while (pos != string::npos) {
      string::size_type p = s.find(delim, pos);

      if (p == string::npos) {
        result.push_back(s.substr(pos));
        break;
      } else {
        result.push_back(s.substr(pos, p - pos));
      }

      pos = p + delim.size();
    }

    // compress
    for (size_t i = 0; i < result.size(); i++) {
      if (result[i] == "" || result[i] == delim) {
        result.erase(result.begin() + i);
        i--;
      }
    }

    return result;
  }
  //-----------------------------------------------------------------------------
  static void getLRFGLdata(const std::vector<LRFPoint> &LRF_data,
                           std::vector<float> &vertices,
                           std::vector<unsigned int> &elements, float height) {
    const double distance_threshold = 30.0;
    vertices.clear();
    vertices.reserve(3 * 4 * 1800);
    elements.clear();
    elements.reserve(3 * 3 * 1800);

    // origin
    vertices.push_back(0.0f);
    vertices.push_back(0.0f);
    vertices.push_back(0.0f);

    for (size_t i = 1; i < LRF_data.size(); i++) {
      double diff_x = LRF_data[i].x - LRF_data[i - 1].x;
      double diff_y = LRF_data[i].y - LRF_data[i - 1].y;
      double distance = sqrt(diff_x * diff_x + diff_y * diff_y);

      int current_num = int(vertices.size() / 3);
      // vertex
      vertices.push_back(LRF_data[i - 1].x);
      vertices.push_back(LRF_data[i - 1].y);
      vertices.push_back(0.0f);

      vertices.push_back(LRF_data[i - 1].x);
      vertices.push_back(LRF_data[i - 1].y);
      vertices.push_back(height);

      vertices.push_back(LRF_data[i].x);
      vertices.push_back(LRF_data[i].y);
      vertices.push_back(height);

      vertices.push_back(LRF_data[i].x);
      vertices.push_back(LRF_data[i].y);
      vertices.push_back(0.0f);

      if (distance < distance_threshold) {
        // element
        elements.push_back(current_num);
        elements.push_back(current_num + 1);
        elements.push_back(current_num + 2);

        elements.push_back(current_num);
        elements.push_back(current_num + 2);
        elements.push_back(current_num + 3);
      }

      // floor
      elements.push_back(0);
      elements.push_back(current_num);
      elements.push_back(current_num + 3);
    }

    // floor loop close
    elements.push_back(0);
    int current_num = int(vertices.size() / 3) - 1;
    elements.push_back(current_num);
    elements.push_back(1);
  }

  static void drawPoints(const std::vector<LRFPoint> &LRF_data, cv::Mat &dst,
                         const int img_width = 600,
                         const double max_dist = 5.0) {
    dst = cv::Mat::zeros(cv::Size(img_width, img_width), CV_8UC3);
    cv::circle(dst, cv::Point2d(img_width / 2, img_width / 2), 3,
               cv::Scalar(0, 255, 0), -1);
    for (const auto &it : LRF_data) {
      int u = int(img_width * it.x / max_dist + img_width / 2.0);
      int v = int(-img_width * it.y / max_dist + img_width / 2.0);
      // point outside image is not drawn
      cv::circle(dst, cv::Point2d(u, v), 3, cv::Scalar(0, 0, 255), -1);
    }
  }

 protected:
  bool isOpened = false;
};
}  // namespace sensor