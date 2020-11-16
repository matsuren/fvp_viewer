#include "fvp/fvp_system.hpp"

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <memory>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/transform.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>

#include "calib/ocam_functions.hpp"
#include "fvp/config.hpp"
#include "fvp/gl_camera.hpp"
#include "fvp/gl_data_manager.hpp"
#include "fvp/gl_model_manager.hpp"
#include "glslcookbook/glutils.h"
#include "models/dome.hpp"
#include "models/mesh.hpp"
#include "models/plane.hpp"

namespace fvp {
System::System(const std::shared_ptr<Config> &config)
    : cfg(config),
      gl_data_mgr(std::make_unique<GLDataManager>()),
      model_mgr(std::make_unique<GLModelManager>()),
      gl_cam_mgr(std::make_unique<GLCamera>()),
      window(nullptr),
      WIN_TITLE("Free viewpoint image generation"),
      win_width(720),
      win_height(540),
      render_mode(RenderMode::LRF),
      is_animating(false),
      exit_flag(false) {
  std::cout << "+++++++++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "+++ Free viewpoint image generation +++" << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++++++++" << std::endl;
}
//-----------------------------------------------------------------------------
int System::initGLFW() {
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
  window =
      glfwCreateWindow(win_width, win_height, WIN_TITLE.c_str(), NULL, NULL);
  if (!window) {
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);

  // Load the OpenGL functions.
  if (!gladLoadGL()) {
    return -1;
  }
  // Doesn't work for NVIDIA GPU
  if (glfwExtensionSupported("GLX_SGI_swap_control") ||
      glfwExtensionSupported("GLX_EXT_swap_control") ||
      glfwExtensionSupported("WGL_EXT_swap_control")) {
    if (glfwExtensionSupported("GLX_EXT_swap_control_tear") ||
        glfwExtensionSupported("WGL_EXT_swap_control_tear"))
      glfwSwapInterval(-1);
    else
      glfwSwapInterval(1);
  }

  glfwSetWindowUserPointer(window, this);
  glfwSetKeyCallback(window, key_callback);
  glfwSetScrollCallback(window, scroll_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, cursor_position_callback);
  // set window size
  glfwSetWindowSizeCallback(window, resize_callback);

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

  resizeGLWindow(win_width, win_height);

  return 0;
}
//-----------------------------------------------------------------------------
void System::initScene() {
  if (window == nullptr) {
    spdlog::error("Not yet initialized. Call initGLFW first");
    throw std::runtime_error("Not yet initialized GLFW");
  }
  const int CAMERA_NUM = cfg->num_camera();
  glEnable(GL_DEPTH_TEST);

  compileAndLinkShader();
  ////////////////////////////////
  // Load GL model
  ////////////////////////////////
  const std::string robot_model_file = cfg->robot_model_filename();
  spdlog::info("Loading: {}", robot_model_file);
  model_mgr->setDrawableModel(
      "robot", std::make_shared<model::Mesh>(robot_model_file, &prog_robot));

  model_mgr->setDrawableModel("dome", std::make_shared<model::Dome>(2.0f, 50));

  const float plane_size = 10.0f;
  model_mgr->setDrawableModel(
      "floor", std::make_shared<model::Plane>(plane_size, plane_size, 1, 1));

  ////////////////////////////////
  // Load model matrix
  ////////////////////////////////
  glm::mat4 tmp_model_mat = glm::mat4(1.0f);
  tmp_model_mat *= glm::translate(glm::vec3(0.0f, 0.0f, 0.0f));
  tmp_model_mat *=
      glm::rotate(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
  model_mgr->setModelMatrix("dome", tmp_model_mat);
  model_mgr->setModelMatrix("floor", tmp_model_mat);

  double lrf_x, lrf_y, lrf_rad;
  tmp_model_mat = glm::mat4(1.0f);
  cfg->getLRFPose(lrf_x, lrf_y, lrf_rad);
  tmp_model_mat *= glm::translate(glm::vec3(lrf_x, lrf_y, 0.0f));
  tmp_model_mat *= glm::rotate(float(lrf_rad), glm::vec3(0.0f, 0.0f, 1.0f));
  model_mgr->setModelMatrix("LRF", tmp_model_mat);

  cv::Mat cv_model_mat;
  cfg->getRobotPose(cv_model_mat);
  model_mgr->setModelMatrix("robot", cv_model_mat);
  prog.use();
  prog.setUniform("CAMERA_NUM", CAMERA_NUM);

  ////////////////////////////////////////
  // Load camera extrinsic and intrinsic
  ////////////////////////////////////////
  const std::string yml_fname = cfg->cam_pose_filename();
  cv::FileStorage fs(yml_fname, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    spdlog::error("Cannot open camera pose yaml file: {}", yml_fname);
    throw std::runtime_error("No camera pose yaml");
  }
  for (int cam_id = 0; cam_id < CAMERA_NUM; cam_id++) {
    // Intrinsic (OCamCalib)
    const std::string ocam_fname = cfg->calib_filenames(cam_id);
    setCameraCalibData(ocam_fname, cam_id);

    // Extrinsic
    const std::string key = fmt::format("img{}origin", cam_id);
    cv::Mat pose;
    fs[key] >> pose;
    cv::Mat tmp_viewmat;
    // transposed (column-wise and row-wise conversion)
    pose.convertTo(tmp_viewmat, CV_32F);
    cv::transpose(tmp_viewmat, tmp_viewmat);
    FisheyeViews[cam_id] = glm::make_mat4(tmp_viewmat.ptr<float>(0, 0));

    // print camera poses
    spdlog::debug("fisheye_views[{}]", cam_id);
    for (int j = 0; j < 4; j++) {
      spdlog::debug(glm::to_string(FisheyeViews[cam_id][j]));
    }
  }

  // Debug glsl
  if (SPDLOG_LEVEL_DEBUG >= spdlog::get_level()) {
    prog.printActiveAttribs();
    prog.printActiveUniformBlocks();
    prog.printActiveUniforms();
    std::cout << std::endl;
  }
  // environment setting
  prog_robot.use();  // Don't foget call prog.use() before call prog.set_uniform
  const glm::vec4 worldLight = glm::vec4(5.0f, 5.0f, -4.0f, 1.0f);
  gl_cam_mgr->setWorldLightPosition(worldLight);
}
//-----------------------------------------------------------------------------
void System::mainLoop() {
  // check all modules are initialized
  if (window == nullptr) {
    spdlog::error("Not yet initialized. Call initGLFW first");
    throw std::runtime_error("GLFW is not yet initialized");
  }
  if (!gl_data_mgr->is_initialized()) {
    spdlog::error("Not yet initialized. Initialize GLDataManager first");
    throw std::runtime_error("GLDataManager is not yet initialized");
  }
  if (model_mgr->getAllModelKeys().size() == 0) {
    spdlog::error("Nothing to render. Call initScene first");
    throw std::runtime_error("GLModelManager is not yet initialized");
  }

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
      strm << WIN_TITLE;
      strm.precision(4);
      strm << " (fps: " << fps << ")";
      glfwSetWindowTitle(window, strm.str().c_str());
    }
  }

  threadExit();
}
//-----------------------------------------------------------------------------
void System::update(float t) {
  gl_cam_mgr->update(t);
  model_mgr->update(t);
  gl_data_mgr->update(t);
}
//-----------------------------------------------------------------------------
void System::render() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // glFrontFace(GL_CW);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  // get camera view matrix
  ViewMatrix = gl_cam_mgr->getViewMat();

  prog.use();

  switch (render_mode) {
    case fvp::RenderMode::FLOOR:
      ModelMatrix = model_mgr->getModelMatrix("floor");
      setMatrices();
      model_mgr->drawModel("floor");
      break;
    case fvp::RenderMode::DOME:
      ModelMatrix = model_mgr->getModelMatrix("dome");
      setMatrices();
      model_mgr->drawModel("dome");
      break;
    case fvp::RenderMode::LRF:
      ModelMatrix = model_mgr->getModelMatrix("LRF");
      setMatrices();
      gl_data_mgr->drawModel("LRF");
      break;
  }

  prog_robot.use();
  // draw robot
  ModelMatrix = model_mgr->getModelMatrix("robot");
  setMatricesPassthrough();
  model_mgr->drawModel("robot");
}
//-----------------------------------------------------------------------------
void System::setMatrices() {
  const glm::mat4 mv = ViewMatrix * ModelMatrix;
  prog.setUniform("ModelMatrix", ModelMatrix);
  prog.setUniform("ModelViewMatrix", mv);
  for (int i = 0; i < FisheyeViews.size(); i++) {
    const std::string uniform_name = fmt::format("FisheyeCameraViews[{}]", i);
    prog.setUniform(uniform_name.c_str(), FisheyeViews[i] * ModelMatrix);
  }
  prog.setUniform("NormalMatrix", glm::mat3(glm::vec3(mv[0]), glm::vec3(mv[1]),
                                            glm::vec3(mv[2])));
  prog.setUniform("MVP", ProjMatrix * mv);
}
//-----------------------------------------------------------------------------
void System::setMatricesPassthrough() {
  prog_robot.setUniform("PointSize", 8.0f);
  ViewMatrix = gl_cam_mgr->getViewMat();
  const glm::mat4 mv = ViewMatrix * ModelMatrix;
  prog_robot.setUniform("ModelViewMatrix", mv);
  prog_robot.setUniform(
      "NormalMatrix",
      glm::mat3(glm::vec3(mv[0]), glm::vec3(mv[1]), glm::vec3(mv[2])));
  prog_robot.setUniform("MVP", ProjMatrix * mv);

  // light
  const glm::vec4 worldLight = gl_cam_mgr->getWorldLightPosition();
  prog_robot.setUniform("Light.Position", ViewMatrix * worldLight);
  prog_robot.setUniform("Light.Ld", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
  prog_robot.setUniform("Light.La", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
  prog_robot.setUniform("Light.Ls", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
}
//-----------------------------------------------------------------------------
void System::resizeGLWindow(int w, int h) {
  constexpr float fov = glm::radians<float>(47.0f);
  glViewport(0, 0, w, h);
  win_width = w;
  win_height = h;
  ProjMatrix = glm::perspective(fov, (float)w / h, 0.3f, 1000.0f);
}
//-----------------------------------------------------------------------------
void System::getGLWinSize(int &w, int &h) {
  w = win_width;
  h = win_height;
}
//-----------------------------------------------------------------------------
void System::setRenderMode(const int int_mode) {
  if (0 < int_mode && int_mode < 4) {
    setRenderMode(static_cast<fvp::RenderMode>(int_mode));
  } else {
    spdlog::warn("Invalid render mode:{}", int_mode);
  }
}
//-----------------------------------------------------------------------------
void System::setRenderMode(const RenderMode mode) { render_mode = mode; }
//-----------------------------------------------------------------------------
void System::compileAndLinkShader() {
  spdlog::info("Compile and link shader");
  try {
    const std::string base = "../../shader/";
    spdlog::info("Loading projtex.[vs, fs] from {}", base);
    prog.compileShader("../../shader/projtex.vs");
    prog.compileShader("../../shader/projtex.fs");
    prog.link();
    prog.use();

    spdlog::info("Loading simpleAssimpShader.[vert, frag] from {}", base);
    prog_robot.compileShader("../../shader/simpleAssimpShader.vert");
    prog_robot.compileShader("../../shader/simpleAssimpShader.frag");
    prog_robot.link();
    // prog_robot.use();
  } catch (GLSLProgramException &e) {
    spdlog::error("GLSLProgramException: {}", e.what());
    exit(EXIT_FAILURE);
  }
}
//-----------------------------------------------------------------------------
void System::setCameraCalibData(const std::string fname, const int cam_id) {
  if (cam_id > cfg->num_camera()) {
    spdlog::error("cam_id is larger than CAMERA_NUM");
    return;
  }
  /* --------------------------------------------------------------------*/
  /* Read the parameters of the omnidirectional camera from the TXT file */
  spdlog::info("Loading calib file: {}", fname);
  struct ocam_model o;
  if (get_ocam_model(&o, fname.c_str()) == -1) {
    spdlog::error("Cannot load calib file:{}", fname);
    throw std::runtime_error("No calib file");
  }
  /* --------------------------------------------------------------------*/
  /* Print ocam_model parameters                                         */
  /* --------------------------------------------------------------------*/
  const std::string param_name = fmt::format("OCamParams[{}]", cam_id);
  const int INVPOL_MAX = 14;
  const int POL_MAX = 6;
  while (o.length_invpol < INVPOL_MAX) {
    o.invpol[o.length_invpol] = 0;
    o.length_invpol++;
  }
  while (o.length_pol < POL_MAX) {
    o.pol[o.length_pol] = 0;
    o.length_pol++;
  }

  spdlog::debug("pol =");
  for (int i = 0; i < o.length_pol; i++) {
    spdlog::debug("\t{:.4e}", o.pol[i]);
  };
  spdlog::debug("invpol =");
  for (int i = 0; i < o.length_invpol; i++) {
    spdlog::debug("\t{:.4e}", o.invpol[i]);
  };
  spdlog::debug("xc={}, yc={}, width={}, height={}", o.xc, o.yc, o.width,
                o.height);
  prog.setUniform(param_name + ".xc", o.xc);
  prog.setUniform(param_name + ".yc", o.yc);
  prog.setUniform(param_name + ".c", o.c);
  prog.setUniform(param_name + ".d", o.d);
  prog.setUniform(param_name + ".e", o.e);
  prog.setUniform(param_name + ".width", float(o.width));
  prog.setUniform(param_name + ".height", float(o.height));
  // set array
  prog.setUniform(param_name + ".invpol", o.invpol, INVPOL_MAX);
  prog.setUniform(param_name + ".pol", o.pol, POL_MAX);
}

// GLFW callback
//-----------------------------------------------------------------------------
void System::key_callback(GLFWwindow *window, int key, int scancode, int action,
                          int mods) {
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
void System::scroll_callback(GLFWwindow *window, double x, double y) {
  System *system_ptr = getPointerFromGLFW(window);
  system_ptr->gl_cam_mgr->scrollCallback(x, y);
}
//-----------------------------------------------------------------------------
void System::mouse_button_callback(GLFWwindow *window, int button, int action,
                                   int mods) {
  System *system_ptr = getPointerFromGLFW(window);
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    double x, y;
    glfwGetCursorPos(window, &x, &y);
    if (action == GLFW_PRESS)
      system_ptr->gl_cam_mgr->mouseCursorCallback(x, y, "PRESS");
    if (action == GLFW_RELEASE)
      system_ptr->gl_cam_mgr->mouseCursorCallback(x, y, "RELEASE");
  }
}
//-----------------------------------------------------------------------------
void System::cursor_position_callback(GLFWwindow *window, double x, double y) {
  System *system_ptr = getPointerFromGLFW(window);
  system_ptr->gl_cam_mgr->mouseCursorCallback(x, y, "CURSOR_MOVE");
}
//-----------------------------------------------------------------------------
void System::resize_callback(GLFWwindow *window, int width, int height) {
  System *system_ptr = getPointerFromGLFW(window);
  system_ptr->resizeGLWindow(width, height);
}
//-----------------------------------------------------------------------------
System *System::getPointerFromGLFW(GLFWwindow *window) {
  return static_cast<System *>(glfwGetWindowUserPointer(window));
}
//-----------------------------------------------------------------------------
void System::error_callback(int error, const char *description) {
  spdlog::error("GLFW Error: {}", description);
}
//-----------------------------------------------------------------------------
int System::saveCapture(const std::string fname) {
  char *buffer;
  int width, height;
  getGLWinSize(width, height);
  const size_t DATA_SIZE = width * height;
  buffer = (char *)calloc(4, DATA_SIZE);
  glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
  cv::Mat img(height, width, CV_8UC4, buffer);
  cv::flip(img, img, 0);
  cv::cvtColor(img, img, cv::COLOR_RGBA2BGR);
  int ret = cv::imwrite(fname, img);
  if (ret) {
    spdlog::info("Screen capture: {}", fname);
  } else {
    spdlog::error("Screen capture error: {}", fname);
  }
  return ret;
}
//-----------------------------------------------------------------------------
bool System::checkExit() { return exit_flag; }
//-----------------------------------------------------------------------------
void System::threadExit() {
  // Close window and terminate GLFW
  glfwTerminate();
  exit_flag = true;
}
}  // namespace fvp
