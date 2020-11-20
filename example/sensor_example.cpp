#include <spdlog/spdlog.h>

#include "fvp/config.hpp"
#include "fvp/fvp_system.hpp"
#include "sensors/sensor_manager.hpp"
#include <opencv2/highgui.hpp>
#include <thread>
using GLuint = unsigned int;
int main(int argc, char *argv[]) {

  // Set logger
  // Runtime log levels
  spdlog::set_level(spdlog::level::info);
  // spdlog::set_level(spdlog::level::trace);
  std::shared_ptr<fvp::System> fvp_system;
  std::unique_ptr<SensorManager> sensor_mgr;
  try {
    const std::string cfg_fname = "../../config_FVP_parameters.json";
    auto cfg = std::make_shared<fvp::Config>(cfg_fname);
    fvp_system = std::make_shared<fvp::System>(cfg);
    sensor_mgr = std::make_unique<SensorManager>(cfg);
    sensor_mgr->setFVPSystem(fvp_system);
    // Initialize GLFW and create window
    fvp_system->initGLFW();
    // Loading sample images
    std::vector<cv::Mat> sample_imgs;
    for (int i = 0; i < cfg->num_camera(); i++) {
      const std::string fname = cfg->image_filenames(i);
      spdlog::info("Loading {}", fname);
      cv::Mat img = cv::imread(fname);
      if (img.empty()) {
        spdlog::error("Cannot open image file:{}", fname);
        throw std::runtime_error("No image file");
      }
      sample_imgs.push_back(img);
    }
    fvp_system->initImages(sample_imgs);

    // Loading sample LRF data
    std::vector<sensor::LRFPoint> LRF_data;
    sensor::BaseLRF::loadLRFDataCSV(cfg->lrf_data_filename(), LRF_data);
    std::vector<float> vertices;
    std::vector<GLuint> elements;
    sensor::BaseLRF::getLRFGLdata(LRF_data, vertices, elements,
                                    sensor_mgr->LRF_wall_height);
    fvp_system->initMesh(vertices, elements);

    // Prepare scene
    fvp_system->initScene();

    // Start capturing sensors
    bool with_imshow = false;
    sensor_mgr->startCapture(with_imshow);

    ///////////////////////////////
    // Enter the main loop
    fvp_system->mainLoop();
    ///////////////////////////////

    // Exit loop

    // Take care of SensorManager
    sensor_mgr->join();

  } catch (const std::exception &e) {
    spdlog::error("Catch exception: {}", e.what());
    // Finish thread
    fvp_system->threadExit();
    // Take care of SensorManager
    sensor_mgr->join();

    return -1;
  }

  // Exit program
  return 0;
}
