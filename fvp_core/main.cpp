#include "main.hpp"

#include <GLFW/glfw3.h>
#include <rplidar.h>

#include <cstdio>
#include <cstdlib>
#include <memory>

#include "gl_camera.hpp"
#include "gl_data_manager.hpp"
#include "config.hpp"
#include "fvp_system.hpp"
#include "glutils.h"
#include "sensors/sensor_manager.hpp"
#include "spdlog/spdlog.h"

//#define WIN_WIDTH 1200
//#define WIN_HEIGHT 900
#define WIN_WIDTH 720
#define WIN_HEIGHT 540

#include <atomic>
#include <iomanip>
#include <sstream>

fvp::System *fvp_system;
GLFWwindow *window;
string title;

// ----- global extern in main.hpp--------
// thread exit
std::atomic<bool> exit_flag(false);
bool checkExit() { return exit_flag; }
void threadExit() { exit_flag = true; }

//-----------------------------------------------------------------------------
static void key_callback(GLFWwindow *window, int key, int scancode, int action,
                         int mods) {
  if (fvp_system == nullptr) return;
  if (key == GLFW_KEY_SPACE && action == GLFW_RELEASE)
    fvp_system->animate(!(fvp_system->animating()));
  if (key >= GLFW_KEY_1 && key <= GLFW_KEY_4 && action == GLFW_PRESS)
	  fvp_system->setRenderMode(key - GLFW_KEY_0);
  if (key == GLFW_KEY_RIGHT && action == GLFW_RELEASE)
    GLCameraManager::getInstance().angle += 0.01f;
  if (key == GLFW_KEY_LEFT && action == GLFW_RELEASE)
    GLCameraManager::getInstance().angle -= 0.01f;
  if (key == GLFW_KEY_P && action == GLFW_RELEASE) {
    char *buffer;
    int width, height;
    fvp_system->getwinsize(width, height);
    buffer = (char *)calloc(4, width * height);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
    cv::Mat img(height, width, CV_8UC4, buffer);
    cv::flip(img, img, 0);
    cv::cvtColor(img, img, cv::COLOR_RGBA2BGR);
    const std::string fname = "capture.jpg";
    cv::imwrite(fname, img);
    std::cout << "Capture screen: " << fname << std::endl;
  }
}
//-----------------------------------------------------------------------------
static void scroll_callback(GLFWwindow *window, double x, double y) {
  GLCameraManager::getInstance().scrollCallback(x, y);
}
//-----------------------------------------------------------------------------
static void mouse_button_callback(GLFWwindow *window, int button, int action,
                                  int mods) {
  double x, y;
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    glfwGetCursorPos(window, &x, &y);
    GLCameraManager::getInstance().pressedLeftButton(x, y);
    // std::cout << "x :" << x << "\t y :" << y << std::endl;
  }
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
    glfwGetCursorPos(window, &x, &y);
    GLCameraManager::getInstance().releasedLeftButton(x, y);
    // std::cout << "x :" << x << "\t y :" << y << std::endl;
  }
}
//-----------------------------------------------------------------------------
static void cursor_position_callback(GLFWwindow *window, double x, double y) {
  GLCameraManager::getInstance().cursorPositionCallback(x, y);
}
//-----------------------------------------------------------------------------
void resizeGL(int w, int h) { fvp_system->resize(w, h); }
//-----------------------------------------------------------------------------
static void resize_callback(GLFWwindow *window, int width, int height) {
  resizeGL(width, height);
}
//-----------------------------------------------------------------------------
void initializeGL() {
  glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
#ifdef _DEBUG
  glDebugMessageCallback(GLUtils::debugCallback, NULL);
  glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL,
                        GL_TRUE);
  glDebugMessageInsert(GL_DEBUG_SOURCE_APPLICATION, GL_DEBUG_TYPE_MARKER, 0,
                       GL_DEBUG_SEVERITY_NOTIFICATION, -1, "Start debugging");
#endif
  fvp_system->initScene();
}
//-----------------------------------------------------------------------------
void mainLoop() {
  const int samples = 100;
  float time[samples];
  int index = 0;

  while (!glfwWindowShouldClose(window) &&
         !glfwGetKey(window, GLFW_KEY_ESCAPE)) {
    GLUtils::checkForOpenGLError(__FILE__, __LINE__);
    fvp_system->update(float(glfwGetTime()));
    fvp_system->render();
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
      strm << title;
      strm.precision(4);
      strm << " (fps: " << fps << ")";
      glfwSetWindowTitle(window, strm.str().c_str());
    }
  }

  threadExit();
}
//-----------------------------------------------------------------------------
/** @brief It displays if an error is raised by the glfw
 */
void error_callback(int error, const char *description) {
  spdlog::error("GLFW Error: {}", description);
}
//-----------------------------------------------------------------------------


int main(int argc, char *argv[]) {
  std::cout << "+++++++++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "+++ Free viewpoint image generation +++" << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++++++++" << std::endl;

  // Set logger
  // Runtime log levels
  spdlog::set_level(spdlog::level::info);
  // spdlog::set_level(spdlog::level::trace);

  std::unique_ptr<SensorManager> sensor_mgr;
  try {
    const std::string cfg_fname = "../../config_FVP_parameters.json";
    auto cfg = std::make_shared<fvp::Config>(cfg_fname);
    fvp_system = new fvp::System(cfg);
    auto manager = std::make_shared<fvp::GLDataManager>(cfg);
    fvp_system->setSensorDataManager(manager);

    sensor_mgr = std::make_unique<SensorManager>(cfg);
    sensor_mgr->setSensorDataManager(manager);

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
    title = "Free viewpoint image generation";
    window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, title.c_str(), NULL, NULL);
    if (!window) {
      glfwTerminate();
      return -1;
    }
    glfwMakeContextCurrent(window);
    // glfwSwapInterval(0)  doesn't wait for refresh
    glfwSwapInterval(1);
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
    initializeGL();
    resizeGL(WIN_WIDTH, WIN_HEIGHT);

    // Enter the main loop
    mainLoop();

    // Take care of SensorManager
    sensor_mgr->join();

    // Close window and terminate GLFW
    glfwTerminate();

  } catch (const std::exception &e) {
    spdlog::error("Catch exception: {}", e.what());
    // Finish thread
    threadExit();
    // Take care of SensorManager
    sensor_mgr->join();
    // Close window and terminate GLFW
    glfwTerminate();
    return -1;
  }

  // Exit program
  return 0;
}
