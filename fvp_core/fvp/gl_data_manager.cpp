// clang-format off
#include "glslcookbook/cookbookogl.h"
#include <GLFW/glfw3.h>
// clang-format on
#include "gl_data_manager.hpp"
#include <spdlog/spdlog.h>

#include <algorithm>  // std::copy
#include <iostream>
#include <iterator>  // std::back_inserter
#include <mutex>
#include <opencv2/core.hpp>


namespace fvp {

GLDataManager::GLDataManager()
    : is_texture_initialized(false), is_mesh_initialized(false) {}

// -----------------------------------
int GLDataManager::initImages(std::vector<cv::Mat> &imgs) {
  spdlog::info("[initImages] initializing textures in GPU");
  // Load texture file to texture array
  glActiveTexture(GL_TEXTURE0);
  glGenTextures(1, &texID);
  glBindTexture(GL_TEXTURE_2D_ARRAY, texID);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

  //////////////////////////
  // load camera images
  //////////////////////////
  const int CAMERA_NUM = int(imgs.size());
  for (int i = 0; i < CAMERA_NUM; i++) {
    capture_imgs.push_back(imgs[i]);
    img_pixel_buffers.push_back(0);
    imgs_update_required.push_back(false);
    mtxs.push_back(new std::mutex());
  }

  // Load image in GPU
  for (int i = 0; i < CAMERA_NUM; i++) {
    cv::Mat mat_data = capture_imgs[i];
    img_width = mat_data.cols;
    img_height = mat_data.rows;

    // allocate memory
    if (i == 0) {
      glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGB, img_width, img_height,
                   CAMERA_NUM, 0, GL_BGR, GL_UNSIGNED_BYTE, nullptr);
      // glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_RGB8, width, height,
      // CAMERA_NUM);
    }

    // create a pixel buffer object. you need to delete them when program
    // exits.
    const int DATA_SIZE = 3 * img_width * img_height;
    GLuint pixel_buffer;
    glGenBuffers(1, &pixel_buffer);
    // setPixelbuffer
    img_pixel_buffers[i] = pixel_buffer;
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pixel_buffer);
    glBufferData(GL_PIXEL_UNPACK_BUFFER, DATA_SIZE, mat_data.data,
                 GL_DYNAMIC_DRAW);
    // send data
    glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, i, img_width, img_height, 1,
                    GL_BGR, GL_UNSIGNED_BYTE, nullptr);

    imgs_update_required[i] = false;
  }

  is_texture_initialized = true;
  return 0;
}

// -----------------------------------

int GLDataManager::updateImages(const cv::Mat &img, const int camera_id) {
  if (!is_texture_initialized) {
    spdlog::warn("[updateImages] Texture is not initialized");
    return 1;
  }
  std::lock_guard<std::mutex> lock(*mtxs[camera_id]);
  // capture_imgs[camera_id] = img.clone();
  capture_imgs[camera_id] = img;
  imgs_update_required[camera_id] = true;
  return 0;
}

int GLDataManager::initMesh(const std::vector<float> &vertices,
                            const std::vector<GLuint> &elements) {
  spdlog::info("[initMesh] initializing mesh in GPU");
  //////////////////////////
  // load mesh from LRF data
  //////////////////////////
  std::lock_guard<std::mutex> lock(mesh_mtx);

  std::copy(vertices.begin(), vertices.end(),
            std::back_inserter(mesh_vertices));
  std::copy(elements.begin(), elements.end(),
            std::back_inserter(mesh_elements));

  mesh_element_num = int(mesh_elements.size());
  glGenVertexArrays(1, &mesh_vao);
  glBindVertexArray(mesh_vao);
  glGenBuffers(2, mesh_buffers);

  glBindBuffer(GL_ARRAY_BUFFER, mesh_buffers[0]);
  glBufferData(GL_ARRAY_BUFFER, mesh_vertices.size() * sizeof(float),
               &mesh_vertices[0], GL_DYNAMIC_DRAW);
  glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0,
                        ((GLubyte *)NULL + (0)));
  glEnableVertexAttribArray(0);  // Vertex position

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_buffers[1]);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_elements.size() * sizeof(GLuint),
               &mesh_elements[0], GL_DYNAMIC_DRAW);

  glBindVertexArray(0);
  mesh_update_required = false;

  is_mesh_initialized = true;
  return 0;
}
int GLDataManager::updateMesh(const std::vector<float> &vertices,
                              const std::vector<GLuint> &elements) {
  if (!is_mesh_initialized) {
    spdlog::warn("[updateMesh] Mesh is not initialized");
    return 1;
  }
  std::lock_guard<std::mutex> lock(mesh_mtx);
  mesh_vertices.clear();
  mesh_vertices.reserve(vertices.size());
  std::copy(vertices.begin(), vertices.end(),
            std::back_inserter(mesh_vertices));
  mesh_elements.clear();
  mesh_elements.reserve(elements.size());
  std::copy(elements.begin(), elements.end(),
            std::back_inserter(mesh_elements));
  mesh_update_required = true;
  return 0;
}

bool GLDataManager::is_initialized() {
  return is_mesh_initialized & is_texture_initialized;
 }

// update
void GLDataManager::update(float t) {
  // texture
  for (size_t i = 0; i < capture_imgs.size(); ++i) {
    if (imgs_update_required[i]) {
      if ((img_height != capture_imgs[i].rows) ||
          (img_width != capture_imgs[i].cols)) {
        spdlog::error("Different image size");
        spdlog::error("Input ({},{})", capture_imgs[i].rows,
                      capture_imgs[i].cols);
        spdlog::error("Expected ({},{})", img_height, img_width);
        throw std::runtime_error("Wrong image size.");
      }

      glBindBuffer(GL_PIXEL_UNPACK_BUFFER, img_pixel_buffers[i]);

      // map the buffer object into client's memory
      // Note that glMapBufferARB() causes sync issue.
      // If GPU is working with this buffer, glMapBufferARB() will wait(stall)
      // for GPU to finish its job. To avoid waiting (stall), you can call
      // first glBufferDataARB() with NULL pointer before glMapBufferARB().
      // If you do that, the previous data in PBO will be discarded and
      // glMapBufferARB() returns a new allocated pointer immediately
      // even if GPU is still working with the previous data.
      const int DATA_SIZE = capture_imgs[i].cols * capture_imgs[i].rows * 3;
      glBufferData(GL_PIXEL_UNPACK_BUFFER, DATA_SIZE, 0, GL_DYNAMIC_DRAW);
      GLubyte *ptr =
          (GLubyte *)glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
      if (ptr) {
        {
          std::lock_guard<std::mutex> lock(*mtxs[i]);
          // update data directly on the mapped buffer
          // updatePixels(ptr, DATA_SIZE);
          memcpy(ptr, capture_imgs[i].data, DATA_SIZE);
        }
        glUnmapBuffer(
            GL_PIXEL_UNPACK_BUFFER);  // release pointer to mapping buffer
      }
      glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, GLint(i),
                      capture_imgs[i].cols, capture_imgs[i].rows, 1, GL_BGR,
                      GL_UNSIGNED_BYTE, nullptr);
      imgs_update_required[i] = false;
    }
  }
  // LRF Data update
  if (mesh_update_required) {
    std::lock_guard<std::mutex> lock(mesh_mtx);
    mesh_element_num = int(mesh_elements.size());
    spdlog::debug("mesh updating: len(mesh_elements) = {}", mesh_element_num);
    glBindVertexArray(mesh_vao);

    glBindBuffer(GL_ARRAY_BUFFER, mesh_buffers[0]);
    glBufferData(GL_ARRAY_BUFFER, mesh_vertices.size() * sizeof(float),
                 &mesh_vertices[0], GL_DYNAMIC_DRAW);
    glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0,
                          ((GLubyte *)NULL + (0)));
    glEnableVertexAttribArray(0);  // Vertex position

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_buffers[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_elements.size() * sizeof(GLuint),
                 &mesh_elements[0], GL_DYNAMIC_DRAW);

    glBindVertexArray(0);

    mesh_update_required = false;
  }
}

//-----------------------------------------------------------------------------
void GLDataManager::drawModel(const std::string model_name) {
  if (model_name == "LRF") {
    glBindVertexArray(mesh_vao);
    glDrawElements(GL_TRIANGLES, mesh_element_num, GL_UNSIGNED_INT,
                   ((GLubyte *)NULL + (0)));
  }
}

}  // namespace fvp
