#pragma once
// clang-format off
#include "glslcookbook/cookbookogl.h"
#include <GLFW/glfw3.h>
// clang-format on
#include <spdlog/spdlog.h>

#include <array>
#include <atomic>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "fvp/gl_camera.hpp"
#include "fvp/gl_data_manager.hpp"
#include "fvp/gl_model_manager.hpp"
#include "glslcookbook/glslprogram.h"

struct GLFWwindow;

namespace fvp {
class Config;
class GLDataManager;
class GLModelManager;
class GLCamera;

enum class RenderMode { FLOOR = 1, DOME, LRF };

class System {
 public:
  System(const std::shared_ptr<Config> &config);

  //////////////////////////////////
  // Interface for GLDataManager
  //////////////////////////////////
  int initImages(std::vector<cv::Mat> &imgs) {
    int ret = gl_data_mgr->initImages(imgs);
    return ret;
  }
  //-----------------------------------------------------------------------------
  int updateImages(const cv::Mat &img, const int camera_id) {
    int ret = gl_data_mgr->updateImages(img, camera_id);
    return ret;
  }
  //-----------------------------------------------------------------------------
  int initMesh(const std::vector<float> &vertices,
               const std::vector<GLuint> &elements) {
    int ret = gl_data_mgr->initMesh(vertices, elements);
    return ret;
  }
  //-----------------------------------------------------------------------------
  int updateMesh(const std::vector<float> &vertices,
                 const std::vector<GLuint> &elements) {
    int ret = gl_data_mgr->updateMesh(vertices, elements);
    return ret;
  }
  //////////////////////////////////


  // Init GLFW and create window
  int initGLFW();
  // Init scene 
  void initScene();
 // GLFW main loop for rendering
  void mainLoop();
  // Screen shot OpenGL window
  int saveCapture(const std::string fname);
  // Check exit flag
  bool checkExit();
  // exit flag on
  void threadExit();

  //////////////////////////////////
  //  GLFW callback
  //////////////////////////////////
  static void key_callback(GLFWwindow *window, int key, int scancode,
                           int action, int mods);
  static void scroll_callback(GLFWwindow *window, double x, double y);
  static void mouse_button_callback(GLFWwindow *window, int button, int action,
                                    int mods);
  static void cursor_position_callback(GLFWwindow *window, double x, double y);
  static void resize_callback(GLFWwindow *window, int width, int height);
  static System *getPointerFromGLFW(GLFWwindow *window);
  static void error_callback(int error, const char *description);

 private:
  // Set camera calibration data
  void setCameraCalibData(const std::string fname, const int cam_id);
  
  void update(float t);
  void render();
  void resizeGLWindow(int, int);
  void getGLWinSize(int &w, int &h);
  void setRenderMode(const int int_mode);
  void setRenderMode(const RenderMode mode);

  void animate(bool value) { is_animating = value; }
  bool animating() { return is_animating; }

  void setMatrices();
  void setMatricesPassthrough();
  void compileAndLinkShader();


 private:
  // Pointers for other modules
  const std::shared_ptr<Config> cfg;
  const std::unique_ptr<GLDataManager> gl_data_mgr;
  const std::unique_ptr<GLModelManager> model_mgr;
  const std::unique_ptr<GLCamera> gl_cam_mgr;

  // GLFW window
  GLFWwindow *window;
  const std::string WIN_TITLE;
  int win_width;
  int win_height;

  // Render mode : FLOOR, DOME, LRF
  RenderMode render_mode;

  // GLSLProgram for shader
  GLSLProgram prog;
  GLSLProgram prog_robot;

  // Matrices used in shader
  glm::mat4 ModelMatrix;
  glm::mat4 ViewMatrix;
  std::array<glm::mat4, 4> FisheyeViews;
  glm::mat4 ProjMatrix;

  // Animation
  bool is_animating;

  // Exit flag
  std::atomic<bool> exit_flag;
};
}  // namespace fvp
