#pragma once
#include <spdlog/spdlog.h>
#include <atomic>
#include <array>
#include <glm/glm.hpp>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "gl_camera.hpp"
#include "gl_data_manager.hpp"
#include "gl_model_manager.hpp"
#include "glslprogram.h"
#include "mesh.hpp"
#include "plane.hpp"
// OpenGL headers
#include "cookbookogl.h"
#include "glutils.h"
using glm::mat4;

namespace fvp {
class Config;
class GLDataManager;
class GLModelManager;

enum class RenderMode { FLOOR = 1, DOME, LRF };

class System {
 private:
  const std::shared_ptr<Config> cfg;
  std::unique_ptr<GLDataManager> gl_data_mgr;
  std::unique_ptr<GLModelManager> model_mgr;
  std::unique_ptr<GLCamera> gl_cam_mgr;
  int win_width;
  int win_height;
  int img_width;
  int img_height;

  GLSLProgram prog;
  GLSLProgram prog_robot;

  mat4 ModelMatrix;
  mat4 ViewMatrix;
  std::array<mat4, 4> FisheyeViews;
  mat4 ProjMatrix;

  void setMatrices();
  void setMatricesPassthrough();
  void compileAndLinkShader();
  void setCameraCalibData(const std::string fname, const int cam_id);

 public:
  System(const std::shared_ptr<Config> &config);

  int initImages(std::vector<cv::Mat> &imgs) {
    int ret = gl_data_mgr->initImages(imgs);
    return ret;
  }
  int updateImages(const cv::Mat &img, const int camera_id) {
    int ret = gl_data_mgr->updateImages(img, camera_id);
    return ret;
  }

  int initMesh(const std::vector<float> &vertices,
               const std::vector<GLuint> &elements) {
    int ret = gl_data_mgr->initMesh(vertices, elements);
    return ret;
  }

  int updateMesh(const std::vector<float> &vertices,
                 const std::vector<GLuint> &elements) {
    int ret = gl_data_mgr->updateMesh(vertices, elements);
    return ret;
  }

  int initGLFW() {
    const int WIN_WIDTH = 720;
    const int WIN_HEIGHT = 540;

    // Initialize GLFW
    if (!glfwInit()) return -1;

    // Select OpenGL 4.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, true);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, true);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);
    // uncomment to remove the title bar
    // glfwWindowHint(GLFW_DECORATED, GL_FALSE);
    // set an error callback
    glfwSetErrorCallback(error_callback);

    // Open the window
    window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, window_title.c_str(), NULL,
                              NULL);
    if (!window) {
      glfwTerminate();
      return -1;
    }
    glfwMakeContextCurrent(window);
    // glfwSwapInterval(0)  doesn't wait for refresh
    glfwSwapInterval(1);

    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, key_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    // set window size
    glfwSetWindowSizeCallback(window, resize_callback);

    // Load the OpenGL functions.
    if (!gladLoadGL()) {
      return -1;
    }

    if (SPDLOG_LEVEL_DEBUG >= spdlog::get_level()) {
      GLUtils::dumpGLInfo();
    }

    // Initialization
    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
#ifdef _DEBUG
    glDebugMessageCallback(GLUtils::debugCallback, NULL);
    glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL,
                          GL_TRUE);
    glDebugMessageInsert(GL_DEBUG_SOURCE_APPLICATION, GL_DEBUG_TYPE_MARKER, 0,
                         GL_DEBUG_SEVERITY_NOTIFICATION, -1, "Start debugging");
#endif
    initScene();
    resizeGL(WIN_WIDTH, WIN_HEIGHT);

    return 0;
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

  //-----------------------------------------------------------------------------
  void mainLoop() {
    const int samples = 100;
    float time[samples];
    int index = 0;

    while (!glfwWindowShouldClose(window) &&
           !glfwGetKey(window, GLFW_KEY_ESCAPE)) {
      GLUtils::checkForOpenGLError(__FILE__, __LINE__);
      update(float(glfwGetTime()));
      render();
      glfwSwapBuffers(window);
      glfwPollEvents();

      // Update FPS
      time[index] = float(glfwGetTime());
      index = (index + 1) % samples;

      if (index == 0) {
        float sum = 0.0f;
        for (int i = 0; i < samples - 1; i++) sum += time[i + 1] - time[i];
        float fps = samples / sum;

        std::stringstream strm;
        strm << window_title;
        strm.precision(4);
        strm << " (fps: " << fps << ")";
        glfwSetWindowTitle(window, strm.str().c_str());
      }
    }

    threadExit();
  }

  // GLFW callback
  //-----------------------------------------------------------------------------
  static void key_callback(GLFWwindow *window, int key, int scancode,
                           int action, int mods) {
    System *system_ptr = getPointerFromGLFW(window);
    if (key == GLFW_KEY_SPACE && action == GLFW_RELEASE)
      system_ptr->animate(!(system_ptr->animating()));
    if (key >= GLFW_KEY_1 && key <= GLFW_KEY_4 && action == GLFW_PRESS)
      system_ptr->setRenderMode(key - GLFW_KEY_0);
    if (key == GLFW_KEY_RIGHT && action == GLFW_RELEASE)
      system_ptr->gl_cam_mgr->diffAngle(0.01f);
    if (key == GLFW_KEY_LEFT && action == GLFW_RELEASE)
      system_ptr->gl_cam_mgr->diffAngle(-0.01f);
    if (key == GLFW_KEY_P && action == GLFW_RELEASE) {
      static int cnt = 0;
      const std::string fname = fmt::format("capture_{}.jpg", cnt);
      system_ptr->saveCapture(fname);
      cnt++;
    }
  }
  //-----------------------------------------------------------------------------
  static void scroll_callback(GLFWwindow *window, double x, double y) {
    System *system_ptr = getPointerFromGLFW(window);
    system_ptr->gl_cam_mgr->scrollCallback(x, y);
  }
  //-----------------------------------------------------------------------------
  static void mouse_button_callback(GLFWwindow *window, int button, int action,
                                    int mods) {
    System *system_ptr = getPointerFromGLFW(window);
    double x, y;
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
      glfwGetCursorPos(window, &x, &y);
      system_ptr->gl_cam_mgr->pressedLeftButton(x, y);
      // std::cout << "x :" << x << "\t y :" << y << std::endl;
    }
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
      glfwGetCursorPos(window, &x, &y);
      system_ptr->gl_cam_mgr->releasedLeftButton(x, y);
      // std::cout << "x :" << x << "\t y :" << y << std::endl;
    }
  }
  //-----------------------------------------------------------------------------
  static void cursor_position_callback(GLFWwindow *window, double x, double y) {
    System *system_ptr = getPointerFromGLFW(window);
    system_ptr->gl_cam_mgr->cursorPositionCallback(x, y);
  }
  //-----------------------------------------------------------------------------
  void resizeGL(int w, int h) { resize(w, h); }
  //-----------------------------------------------------------------------------
  static void resize_callback(GLFWwindow *window, int width, int height) {
    System *system_ptr = getPointerFromGLFW(window);
    system_ptr->resizeGL(width, height);
  }

  static System *getPointerFromGLFW(GLFWwindow *window) {
    return static_cast<System *>(glfwGetWindowUserPointer(window));
  }

  static void error_callback(int error, const char *description) {
    spdlog::error("GLFW Error: {}", description);
  }
  int saveCapture(const std::string fname);
  bool checkExit() { return exit_flag; }
  void threadExit() { exit_flag = true; }

 private:
  GLFWwindow *window;
  const std::string window_title;

 protected:
  bool is_animating;
  RenderMode render_mode;
  const int CAMERA_NUM;
  std::atomic<bool> exit_flag = false;
};
}  // namespace fvp
