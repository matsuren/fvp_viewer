#pragma once
#include <array>
#include <glm/glm.hpp>
#include <memory>
#include <opencv2/core.hpp>

#include "glslprogram.h"
#include "mesh.hpp"
#include "plane.hpp"
// OpenGL headers
#include "cookbookogl.h"

using glm::mat4;

namespace fvp {
class Config;
class GLDataManager;
class GLModelManager;

enum class RenderMode { FLOOR = 1, DOME, LRF};

class System {
 private:
  const std::shared_ptr<Config> cfg;
  std::shared_ptr<GLDataManager> gl_data_mgr;
  std::unique_ptr<GLModelManager> model_mgr;
  int win_width;
  int win_height;
  int img_width;
  int img_height;

  GLSLProgram prog;
  GLSLProgram prog_robot;

  mat4 ModelMatrix;
  mat4 ViewMatrix;
  std::array<mat4, 4> FisheyeViews;
  std::array<cv::Mat, 4> fisheye_views_cvmat;
  mat4 ProjMatrix;

  void setMatrices();
  void setMatricesPassthrough();
  void compileAndLinkShader();
  void setCameraCalibData(const std::string fname, const int cam_id);

 public:
  System(const std::shared_ptr<Config> &config);

  void setSensorDataManager(std::shared_ptr<GLDataManager> &manager) {
    gl_data_mgr = manager;
  }
  void initScene();
  void update(float t);
  void render();
  void resize(int, int);
  void getwinsize(int &w, int &h);
  void setRenderMode(const int int_mode);
  void setRenderMode(const RenderMode mode);

  void animate(bool value) { is_animating = value; }
  bool animating() { return is_animating; }

 protected:
  bool is_animating;
  RenderMode render_mode;
  const int CAMERA_NUM;
};
}  // namespace fvp
