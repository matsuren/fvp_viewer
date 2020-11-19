#include "fvp_viewer/fvp_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <spdlog/spdlog.h>
#include <tf/transform_listener.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fvp/config.hpp>
#include <fvp/fvp_system.hpp>
#include <opencv2/highgui.hpp>

using GLuint = unsigned int;
class LaserScanListenerNode {
 protected:
  // Our NodeHandle
  ros::NodeHandle _nh;

  // Subscriber for scan data
  ros::Subscriber _sub;

  // main fvp_system
  std::shared_ptr<fvp::System> _system;

  // wall height
  const float _wall_height;

 public:
  // Constructor
  LaserScanListenerNode(std::shared_ptr<fvp::System> &fvp)
      : _system(fvp), _wall_height(3.0) {
    _sub = _nh.subscribe("scan", 1000, &LaserScanListenerNode::callback, this);
  }

  // Callback
  void callback(const sensor_msgs::LaserScan::ConstPtr &msg_in) {
    ROS_DEBUG("Receive scan");
    std::vector<LRFPoint> LRF_data;
    LRF_data.clear();
    LRF_data.reserve(2000);
    const int range_size = msg_in->ranges.size();
    for (int i = 0; i < range_size; ++i) {
      float angle = msg_in->angle_min + (i * msg_in->angle_increment);
      float dist = msg_in->ranges[i];
      // Check if distance is valid
      if (dist < msg_in->range_min || dist > msg_in->range_max) continue;
      float x = dist * sin(angle);
      float y = -dist * cos(angle);
      LRF_data.emplace_back(x, y);
    }

    std::vector<float> vertices;
    std::vector<GLuint> elements;

    getLRFGLdata(LRF_data, vertices, elements, _wall_height);

    _system->updateMesh(vertices, elements);
  }
};
class ImageListenerNode {
  ros::NodeHandle _nh;
  const int _cam_id;
  image_transport::ImageTransport _it;
  image_transport::Subscriber _sub;

  // main fvp_system
  std::shared_ptr<fvp::System> _system;

 public:
  ImageListenerNode(std::shared_ptr<fvp::System> &fvp, const int cam_id)
      : _it(_nh), _system(fvp), _cam_id(cam_id) {
    const std::string topic_name = fmt::format("/camera{}/image_raw", _cam_id);

    // Subscribe images
    _sub = _it.subscribe(topic_name, 1, &ImageListenerNode::callback, this);
  }

  ~ImageListenerNode() {}

  void callback(const sensor_msgs::ImageConstPtr &msg) {
    spdlog::debug("image callback: {}", _cam_id);
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // TODO set roi in camera driver
    cv::Rect roi(224, 224, 1600, 1600);
    // Update texture in fvp_system
    cv::Mat roi_img = cv_ptr->image(roi).clone();
    _system->updateImages(roi_img, _cam_id);
  }
};

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
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fvp_node");
  ros::NodeHandle pnh("~");

  // Set logger
  // Runtime log levels
  spdlog::set_level(spdlog::level::info);
  //   spdlog::set_level(spdlog::level::trace);
  std::shared_ptr<fvp::System> fvp_system;
  try {
    std::string cfg_fname;
    pnh.param<std::string>("config", cfg_fname, "config_FVP_parameters.json");
    auto cfg = std::make_shared<fvp::Config>(cfg_fname);
    fvp_system = std::make_shared<fvp::System>(cfg);

    // Start rendering
    std::thread th(render_worker, std::ref(fvp_system), std::ref(cfg));

    /////////////////////////////////////////////
    // ROS Node Listener
    LaserScanListenerNode scan_node(fvp_system);
    std::vector<ImageListenerNode> image_nodes;
    // TODO add proper deconstructor in fvp_core?
    // You need to reserve first to prevent from calling deconstructor
    image_nodes.reserve(cfg->num_camera());
    for (int cam_id = 0; cam_id < cfg->num_camera(); ++cam_id) {
      image_nodes.emplace_back(fvp_system, cam_id);
    }
    //    ImageListenerNode image_node_0(_system, 0);
    //    ImageListenerNode image_node_1(_system, 1);

    ros::MultiThreadedSpinner spinner(5);  // Use multiple threads
    spinner.spin();  // spin() will not return until the node has been shutdown
    th.join();
  } catch (const std::exception &e) {
    spdlog::error("Catch exception: {}", e.what());
    // Finish thread
    fvp_system->threadExit();
    ros::shutdown();
    return -1;
  }

  // Exit program
  return 0;
}