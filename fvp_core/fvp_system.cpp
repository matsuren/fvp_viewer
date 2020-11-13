#include "fvp_system.hpp"

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "config.hpp"
#include "gl_data_manager.hpp"
#include "glutils.h"
#include "main.hpp"

using glm::vec3;

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/transform.hpp>

#include "gl_camera.hpp"
#include "gl_model_manager.hpp"
#include "models/dome.hpp"

namespace fvp {
System::System(const std::shared_ptr<Config> &config)
    : cfg(config),
      is_animating(false),
      render_mode(RenderMode::LRF),
      CAMERA_NUM(cfg->num_camera()) {
  model_mgr = std::make_unique<GLModelManager>();
}
//-----------------------------------------------------------------------------
void System::initScene() {
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
  mat4 tmp_model_mat = mat4(1.0f);
  tmp_model_mat *= glm::translate(vec3(0.0f, 0.0f, 0.0f));
  tmp_model_mat *= glm::rotate(glm::radians(90.0f), vec3(1.0f, 0.0f, 0.0f));
  model_mgr->setModelMatrix("dome", tmp_model_mat);
  model_mgr->setModelMatrix("floor", tmp_model_mat);

  double lrf_x, lrf_y, lrf_rad;
  tmp_model_mat = mat4(1.0f);
  cfg->getLRFPose(lrf_x, lrf_y, lrf_rad);
  tmp_model_mat *= glm::translate(vec3(lrf_x, lrf_y, 0.0f));
  tmp_model_mat *= glm::rotate(float(lrf_rad), vec3(0.0f, 0.0f, 1.0f));
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
  const vec4 worldLight = vec4(5.0f, 5.0f, -4.0f, 1.0f);
  GLCameraManager::getInstance().setWorldLightPosition(worldLight);


  
  
  // Load sample images into GPU
  gl_data_mgr->initializeFisheye(prog);
  gl_data_mgr->initializeLRF();
}
//-----------------------------------------------------------------------------
void System::update(float t) {
  GLCameraManager::getInstance().update(t);
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
  ViewMatrix = GLCameraManager::getInstance().getViewMat();

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
  const mat4 mv = ViewMatrix * ModelMatrix;
  prog.setUniform("ModelMatrix", ModelMatrix);
  prog.setUniform("ModelViewMatrix", mv);
  for (int i = 0; i < CAMERA_NUM; i++) {
    const std::string uniform_name = fmt::format("FisheyeCameraViews[{}]", i);
    prog.setUniform(uniform_name.c_str(), FisheyeViews[i] * ModelMatrix);
  }
  prog.setUniform("NormalMatrix", mat3(vec3(mv[0]), vec3(mv[1]), vec3(mv[2])));
  prog.setUniform("MVP", ProjMatrix * mv);
}
//-----------------------------------------------------------------------------
void System::setMatricesPassthrough() {
  prog_robot.setUniform("PointSize", 8.0f);
  ViewMatrix = GLCameraManager::getInstance().getViewMat();
  const mat4 mv = ViewMatrix * ModelMatrix;
  prog_robot.setUniform("ModelViewMatrix", mv);
  prog_robot.setUniform("NormalMatrix",
                        mat3(vec3(mv[0]), vec3(mv[1]), vec3(mv[2])));
  prog_robot.setUniform("MVP", ProjMatrix * mv);

  // light
  const vec4 worldLight =
      GLCameraManager::getInstance().getWorldLightPosition();
  prog_robot.setUniform("Light.Position", ViewMatrix * worldLight);
  prog_robot.setUniform("Light.Ld", vec4(1.0f, 1.0f, 1.0f, 1.0f));
  prog_robot.setUniform("Light.La", vec4(1.0f, 1.0f, 1.0f, 1.0f));
  prog_robot.setUniform("Light.Ls", vec4(1.0f, 1.0f, 1.0f, 1.0f));
}
//-----------------------------------------------------------------------------
void System::resize(int w, int h) {
  constexpr float fov = glm::radians<float>(47.0f);
  glViewport(0, 0, w, h);
  win_width = w;
  win_height = h;
  ProjMatrix = glm::perspective(fov, (float)w / h, 0.3f, 1000.0f);
}
//-----------------------------------------------------------------------------
void System::getwinsize(int &w, int &h) {
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
  if (cam_id > CAMERA_NUM) {
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
}  // namespace fvp
