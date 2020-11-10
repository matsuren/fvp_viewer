
#include <array>
#include <fstream>
#include <iostream>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "LRFSensor.hpp"
#include "glslprogram.h"
#include "ocam_functions.hpp"
#include "utils/FpsDisplayer.hpp"
#include "utils/SettingParameters.hpp"

namespace fvp {
class GLDataManager {
 private:
  const std::shared_ptr<Config> cfg;
  int img_width, img_height;
  GLuint texID;

  // Triangle mesh from sensors such as LRF
  std::vector<float> triangle_mesh_vertices;
  std::vector<GLuint> triangle_mesh_elements;

 public:
  bool is_initialized;

  GLDataManager(const std::shared_ptr<Config> &config)
      : cfg(config), is_initialized(false) {}

  // -----------------------------------
  int initializeFisheye(GLSLProgram &prog) {
    const int CAMERA_NUM = int(cfg->num_camera());
    prog.setUniform("CAMERA_NUM", CAMERA_NUM);

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
    for (int i = 0; i < CAMERA_NUM; i++) {
      const std::string fname = cfg->image_filenames(i);
      cv::Mat tmp = cv::imread(fname);
      if (tmp.empty()) {
        std::cout << "cannot load image : " << fname << std::endl;
        return -1;
      }
      capture_imgs.push_back(tmp);
      img_pixel_buffers.push_back(0);
      fisheye_views.push_back(cv::Mat::eye(4, 4, CV_32FC1));
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
      GLuint pixel_buffer;
      glGenBuffers(1, &pixel_buffer);
      setPixelbuffer(pixel_buffer, i);
      glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pixel_buffer);
      glBufferData(GL_PIXEL_UNPACK_BUFFER, 3 * img_width * img_height,
                   mat_data.data, GL_DYNAMIC_DRAW);
      // send data
      glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, i, img_width, img_height, 1,
                      GL_BGR, GL_UNSIGNED_BYTE, nullptr);

      imgs_update_required[i] = false;
    }

    // for OCamCalib
    for (int i = 0; i < CAMERA_NUM; i++) {
      /* --------------------------------------------------------------------*/
      /* Read the parameters of the omnidirectional camera from the TXT file */
      /* --------------------------------------------------------------------*/
      struct ocam_model
          o;  // our ocam_models for the fisheye and catadioptric cameras
              // get_ocam_model(&o, DATA_PATH"/data/calib_results_fisheye.txt");
      std::string ocam_fname = cfg->calib_filenames(i);
      if (get_ocam_model(&o, ocam_fname.c_str()) == -1) {
        exit(EXIT_FAILURE);
      }

      /* --------------------------------------------------------------------*/
      /* Print ocam_model parameters                                         */
      /* --------------------------------------------------------------------*/
      std::string ocamparam_name = "OCamParams[" + std::to_string(i) + "]";
      std::cout << "OcamParam name : " << ocamparam_name << std::endl;
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

      printf("pol =\n");
      for (int i = 0; i < o.length_pol; i++) {
        printf("\t%e\n", o.pol[i]);
      };
      printf("\n");
      printf("invpol =\n");
      for (int i = 0; i < o.length_invpol; i++) {
        printf("\t%e\n", o.invpol[i]);
      };
      printf("\n");
      printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n", o.xc, o.yc,
             o.width, o.height);
      prog.setUniform(ocamparam_name + ".xc", o.xc);
      prog.setUniform(ocamparam_name + ".yc", o.yc);
      prog.setUniform(ocamparam_name + ".c", o.c);
      prog.setUniform(ocamparam_name + ".d", o.d);
      prog.setUniform(ocamparam_name + ".e", o.e);
      prog.setUniform(ocamparam_name + ".width", float(o.width));
      prog.setUniform(ocamparam_name + ".height", float(o.height));
      // set array
      prog.setUniform(ocamparam_name + ".invpol", o.invpol, INVPOL_MAX);
      prog.setUniform(ocamparam_name + ".pol", o.pol, POL_MAX);

      // set fov
      // prog.setUniform(ocamparam_name + ".fov", M_PI);
    }

    is_initialized = true;
    return 0;
  }

  int initializeLRF() {
    //////////////////////////
    // load LRF data
    //////////////////////////
    std::lock_guard<std::mutex> lock(LRF_mtx);
    LRFSensor::loadLRFDataCSV(cfg->lrf_data_filename(), LRF_data);
    LRFSensor::getLRFGLdata(LRF_data, triangle_mesh_vertices,
                            triangle_mesh_elements, LRF_model_height);

    LRF_vertices_num = int(triangle_mesh_vertices.size());
    glGenVertexArrays(1, &LRF_vaoHandle);
    glBindVertexArray(LRF_vaoHandle);
    glGenBuffers(2, LRF_handle);

    glBindBuffer(GL_ARRAY_BUFFER, LRF_handle[0]);
    glBufferData(GL_ARRAY_BUFFER, triangle_mesh_vertices.size() * sizeof(float),
                 &triangle_mesh_vertices[0], GL_DYNAMIC_DRAW);
    glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0,
                          ((GLubyte *)NULL + (0)));
    glEnableVertexAttribArray(0);  // Vertex position

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, LRF_handle[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 triangle_mesh_elements.size() * sizeof(GLuint),
                 &triangle_mesh_elements[0], GL_DYNAMIC_DRAW);

    glBindVertexArray(0);
    LRF_update_required = false;

    is_initialized = true;
    return 0;
  }

  // -----------------------------------
  // get camera image
  int getCameraImage(cv::Mat &ret_mat, const int camera_id) {
    if (static_cast<size_t>(camera_id) < capture_imgs.size()) {
      ret_mat = capture_imgs[camera_id];
      return 0;
    } else {
      std::cout << "error! capture_imgs[" << camera_id << "] is nothing!"
                << std::endl;
      return -1;
    }
  }
  // -----------------------------------
  // get camera image
  int getCameraNumber() { return static_cast<int>(capture_imgs.size()); }
  // -----------------------------------
  // get camera image
  int setPixelbuffer(const GLuint pixel_buffer, const int camera_id) {
    if (static_cast<size_t>(camera_id) < capture_imgs.size()) {
      img_pixel_buffers[camera_id] = pixel_buffer;
      return 0;
    } else {
      std::cout << "error! img_pixel_buffers[" << camera_id << "] is nothing!"
                << std::endl;
      return -1;
    }
  }
  // -----------------------------------
  // get camera image
  GLuint getPixelbuffer(const int camera_id) {
    if (static_cast<size_t>(camera_id) < capture_imgs.size()) {
      return img_pixel_buffers[camera_id];
    } else {
      std::cout << "error! img_pixel_buffers[" << camera_id << "] is nothing!"
                << std::endl;
      return -1;
    }
  }
  // -----------------------------------
  // transposed (the source array is stored column-wise)
  int getCameraViewMatrix(cv::Mat &ret_mat, const int camera_id) {
    if (static_cast<size_t>(camera_id) < fisheye_views.size()) {
      cv::transpose(fisheye_views[camera_id], ret_mat);
      return 0;
    } else {
      std::cout << "error! capture_imgs[" << camera_id << "] is nothing!"
                << std::endl;
      return -1;
    }
  }
  // -----------------------------------
  int setCameraViewMatrix(const cv::Mat &Rt, const int camera_id) {
    if (static_cast<size_t>(camera_id) < fisheye_views.size()) {
      Rt.convertTo(fisheye_views[camera_id], CV_32F);
    } else {
      std::cout << "setCameraViewMatrix error!\n capture_imgs[" << camera_id
                << "] is nothing!" << std::endl;
      return -1;
    }
    return 0;
  }
  // -----------------------------------

  // update
  void update(float t) {
    // texture
    for (size_t i = 0; i < capture_imgs.size(); ++i) {
      if (imgs_update_required[i]) {
        if ((img_height != capture_imgs[i].rows) ||
            (img_width != capture_imgs[i].cols)) {
          std::cout << "Different image size: input (" << capture_imgs[i].rows
                    << ", " << capture_imgs[i].cols;
          std::cout << ") Expected (" << img_height << ", " << img_width << ")"
                    << std::endl;
          exit(EXIT_FAILURE);
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
        int SRC_DATA_SIZE = capture_imgs[i].cols * capture_imgs[i].rows * 3;
        glBufferData(GL_PIXEL_UNPACK_BUFFER, SRC_DATA_SIZE, 0, GL_DYNAMIC_DRAW);
        GLubyte *ptr =
            (GLubyte *)glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
        if (ptr) {
          {
            std::lock_guard<std::mutex> lock(*mtxs[i]);
            // update data directly on the mapped buffer
            // updatePixels(ptr, DATA_SIZE);
            memcpy(ptr, capture_imgs[i].data, SRC_DATA_SIZE);
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
    if (LRF_update_required) {
      std::lock_guard<std::mutex> lock(LRF_mtx);
      LRFSensor::getLRFGLdata(LRF_data, triangle_mesh_vertices,
                              triangle_mesh_elements, LRF_model_height);
      LRF_vertices_num = int(triangle_mesh_vertices.size());
      glBindVertexArray(LRF_vaoHandle);

      glBindBuffer(GL_ARRAY_BUFFER, LRF_handle[0]);
      glBufferData(GL_ARRAY_BUFFER,
                   triangle_mesh_vertices.size() * sizeof(float),
                   &triangle_mesh_vertices[0], GL_DYNAMIC_DRAW);
      glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0,
                            ((GLubyte *)NULL + (0)));
      glEnableVertexAttribArray(0);  // Vertex position

      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, LRF_handle[1]);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                   triangle_mesh_elements.size() * sizeof(GLuint),
                   &triangle_mesh_elements[0], GL_DYNAMIC_DRAW);

      glBindVertexArray(0);

      LRF_update_required = false;
    }
  }

  //-----------------------------------------------------------------------------
  void drawModel(const std::string model_name) {
    if (model_name == "LRF") {
      glBindVertexArray(LRF_vaoHandle);
      glDrawElements(GL_TRIANGLES, LRF_vertices_num, GL_UNSIGNED_INT,
                     ((GLubyte *)NULL + (0)));
    }
  }

 public:
  // setting parameters
  fvp::Config *cfg_;

 private:
  // camera image
  std::vector<cv::Mat> capture_imgs;
  std::vector<GLuint> img_pixel_buffers;
  std::vector<cv::Mat> fisheye_views;
  std::vector<bool> imgs_update_required;
  std::vector<std::mutex *> mtxs;

  // LRF data
  std::vector<LRFPoint> LRF_data;
  float LRF_model_height = 3.0f;
  int LRF_vertices_num = 0;
  unsigned int LRF_vaoHandle;
  unsigned int LRF_handle[2];
  std::mutex LRF_mtx;
  bool LRF_update_required;
};

}  // namespace fvp