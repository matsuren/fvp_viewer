#pragma once
#include <spdlog/spdlog.h>

#include <mutex>
#include <opencv2/core.hpp>

using GLuint = unsigned int;
namespace fvp {
class GLDataManager {
 private:
  int img_width, img_height;
  GLuint texID;

  // camera image
  std::vector<cv::Mat> capture_imgs;
  std::vector<GLuint> img_pixel_buffers;
  std::vector<bool> imgs_update_required;
  std::vector<std::mutex *> mtxs;
  bool is_texture_initialized;

  // Mesh data (e.g., from LRF data)
  int mesh_vertices_num = 0;
  unsigned int mesh_vao;
  unsigned int mesh_buffers[2];
  std::mutex mesh_mtx;
  bool mesh_update_required;

  // Triangle mesh from sensors such as LRF
  std::vector<float> mesh_vertices;
  std::vector<GLuint> mesh_elements;
  bool is_mesh_initialized;

 public:
  GLDataManager();

  int initImages(std::vector<cv::Mat> &imgs);

  int updateImages(const cv::Mat &img, const int camera_id);

  int initMesh(const std::vector<float> &vertices,
               const std::vector<GLuint> &elements);
  int updateMesh(const std::vector<float> &vertices,
                 const std::vector<GLuint> &elements);
  bool is_initialized();
  // update
  void update(float t);
  //-----------------------------------------------------------------------------
  void drawModel(const std::string model_name);
};
}  // namespace fvp
