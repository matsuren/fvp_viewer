#include "gl_model_manager.hpp"

#include <spdlog/spdlog.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <map>
#include <opencv2/core.hpp>

#include "models/drawable.hpp"

namespace fvp {
GLModelManager::GLModelManager(){};

// update
void GLModelManager::update(float t) {
  // if you want use animation for model
  // here is where you will update model
}

void GLModelManager::drawAllModels() {
  for (const auto &it : gl_models) {
    it.second->render();
  }
}
void GLModelManager::drawModel(const std::string key) {
  gl_models[key]->render();
}
void GLModelManager::setDrawableModel(const std::string key,
                                      std::shared_ptr<model::Drawable> object) {
  spdlog::info("setDrawableModel key:{}", key);
  gl_models[key] = object;
}

std::shared_ptr<model::Drawable> GLModelManager::getDrawableModel(
    const std::string key) {
  return gl_models[key];
}

std::vector<std::string> GLModelManager::getAllModelKeys() {
  std::vector<std::string> keys;
  for (auto const &it : gl_models) {
    keys.push_back(it.first);
  }
  return keys;
}

void GLModelManager::setModelMatrix(const std::string key,
                                    const cv::Mat &cv_model_matrix) {
  cv::Mat tmp_viewmat;
  cv_model_matrix.convertTo(tmp_viewmat, CV_32F);
  // transposed
  // OpenCV stores the data in row-major order in memory,
  // but OpenGL mat should be stored in column-major order
  cv::transpose(tmp_viewmat, tmp_viewmat);
  setModelMatrix(key, glm::make_mat4(tmp_viewmat.ptr<float>(0, 0)));
}
void GLModelManager::setModelMatrix(const std::string key,
                                    const glm::mat4 &model_matrix) {
  spdlog::info("setModelMatrix key:{}", key);
  model_matrices[key] = model_matrix;
}

glm::mat4 GLModelManager::getModelMatrix(const std::string key) {
  return model_matrices[key];
}

}  // namespace fvp
