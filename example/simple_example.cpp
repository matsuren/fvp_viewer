#include <spdlog/spdlog.h>

#include <fvp/config.hpp>
#include <fvp/fvp_system.hpp>
#include <opencv2/highgui.hpp>
#include <thread>
using GLuint = unsigned int;

void render_worker(std::shared_ptr<fvp::System> &fvp_system,
            std::shared_ptr<fvp::Config> &cfg) {
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
  const int tmp_width = 3.0;
  std::vector<float> vertices = {
      0.0f, 2 * tmp_width, 0.0f,       -tmp_width, -tmp_width,
      0.0f, tmp_width,     -tmp_width, 0.0f,
  };
  std::vector<GLuint> elements = {0, 1, 2};

  fvp_system->initMesh(vertices, elements);

  // Prepare scene
  fvp_system->initScene();

  ///////////////////////////////
  // Enter the main loop
  fvp_system->mainLoop();
  ///////////////////////////////
}

int main(int argc, char *argv[]) {
  // Set logger
  // Runtime log levels
  spdlog::set_level(spdlog::level::info);
  // spdlog::set_level(spdlog::level::trace);
  std::shared_ptr<fvp::System> fvp_system;
  try {
    const std::string cfg_fname = "../../config_FVP_parameters.json";
    auto cfg = std::make_shared<fvp::Config>(cfg_fname);
    fvp_system = std::make_shared<fvp::System>(cfg);

    std::thread th(render_worker, std::ref(fvp_system), std::ref(cfg));
    std::this_thread::sleep_for(std::chrono::seconds(5));
    // Update test
    for (int i = 0; i < cfg->num_camera(); i++) {
      const std::string fname = cfg->image_filenames(i);
      spdlog::info("Loading {}", fname);
      cv::Mat img = cv::imread(fname);
      if (img.empty()) {
        spdlog::error("Cannot open image file:{}", fname);
        throw std::runtime_error("No image file");
      }
      fvp_system->updateImages(img, i);
    }

    th.join();
  } catch (const std::exception &e) {
    spdlog::error("Catch exception: {}", e.what());
    // Finish thread
    fvp_system->threadExit();
    return -1;
  }

  // Exit program
  return 0;
}
