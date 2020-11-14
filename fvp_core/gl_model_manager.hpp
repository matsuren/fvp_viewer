#pragma once
#include <glm/glm.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/core.hpp>

namespace model {
class Drawable;
}

namespace fvp {
class GLModelManager {
 public:
  GLModelManager();

  // if you want use animation for model
  // here is where you will update model
  void update(float t);

  // Draw all models that are registered
  void drawAllModels();

  // Draw model by given key
  void drawModel(const std::string key);

  // Set drawable model using key
  void setDrawableModel(const std::string key,
                        std::shared_ptr<model::Drawable> object);

  // Get drawable model using key
  std::shared_ptr<model::Drawable> getDrawableModel(const std::string key);

  // Get all drawable model keys
  std::vector<std::string> getAllModelKeys();

  // Set ModelMatrix (cv::Mat or glm::mat4) using key
  void setModelMatrix(const std::string key, const cv::Mat &cv_model_matrix);
  void setModelMatrix(const std::string key, const glm::mat4 &model_matrix);
  // Get ModelMatrix using key
  glm::mat4 getModelMatrix(const std::string key);

 private:
  std::map<std::string, std::shared_ptr<model::Drawable>> gl_models;
  std::map<std::string, glm::mat4> model_matrices;
};

}  // namespace fvp
