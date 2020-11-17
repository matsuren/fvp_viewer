#pragma once
#include <vector>
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

void getLRFGLdata(const std::vector<LRFPoint> &LRF_data,
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

  int current_num = 0;
  for (int i = 0; i < LRF_data.size(); i++) {
    current_num = int(vertices.size() / 3);

    // vertex : current_num
    vertices.push_back(LRF_data[i].x);
    vertices.push_back(LRF_data[i].y);
    vertices.push_back(0.0f);
    // vertex : current_num + 1
    vertices.push_back(LRF_data[i].x);
    vertices.push_back(LRF_data[i].y);
    vertices.push_back(height);

    // No element when i == 0
    if (i == 0) continue;

    // element for wall
    elements.push_back(current_num);
    elements.push_back(current_num - 2);
    elements.push_back(current_num - 1);

    elements.push_back(current_num);
    elements.push_back(current_num - 1);
    elements.push_back(current_num + 1);

    // element for floor
    elements.push_back(0);
    elements.push_back(current_num - 2);
    elements.push_back(current_num);
  }

  // wall loop close
  elements.push_back(1);
  elements.push_back(current_num);
  elements.push_back(current_num + 1);

  elements.push_back(1);
  elements.push_back(current_num + 1);
  elements.push_back(2);

  // floor loop close
  elements.push_back(0);
  elements.push_back(current_num);
  elements.push_back(1);
}
