#include <glm/glm.hpp>
#include <iostream>
#include <map>
using glm::mat4;
#include <opencv2/core.hpp>

#include "models/drawable.hpp"

namespace fvp {
class GLModelManager {
 public:
  GLModelManager(){};

  // update
  void update(float t) {
    // if you want use animation for model
    // here is where you will update model
  }

  void drawAllModels() {
    for (const auto &it : gl_models) {
      it.second->render();
    }
  }
  void drawModel(const std::string key) { gl_models[key]->render(); }
  void setDrawableModel(const std::string key, model::Drawable *object) {
    gl_models[key] = object;
  }

  void setModelMatrix(const std::string key, const cv::Mat &cv_model_matrix) {
    cv::Mat tmp_viewmat;
    cv_model_matrix.convertTo(tmp_viewmat, CV_32F);
    // transposed
    // OpenCV stores the data in row-major order in memory,
    // but OpenGL mat should be stored in column-major order
    cv::transpose(tmp_viewmat, tmp_viewmat);
    setModelMatrix(key, glm::make_mat4(tmp_viewmat.ptr<float>(0, 0)));
  }
  void setModelMatrix(const std::string key, const mat4 &model_matrix) {
    model_matrices[key] = model_matrix;
  }

  mat4 getModelMatrix(const std::string key) { return model_matrices[key]; }

 private:
  std::map<std::string, model::Drawable *> gl_models;
  std::map<std::string, mat4> model_matrices;
};

}  // namespace fvp
