#pragma once
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

namespace fvp {
struct ConfigData {
 public:
  //
  ConfigData();
  template <class Archive>
  void serialize(Archive &archive);

  // data folder that contains calibrated data.
  // ex. img*.jpg, calib_results_*.txt, final_camera_poses.yml
  std::string calib_folder;
  std::string shader_folder;
  std::vector<std::string> image_sources;

  int capture_framerate;
  // LRF COM port
  std::string LRF_com_port;

  std::string ocamcalib_files;
  std::string sample_image_files;
  std::string camera_pose_yml;
  std::string robot_model_file;
  std::string robot_align_yml;
  std::string lrf_align_yml;
  std::string sample_lrf_file;
};

class Config {
 private:
  ConfigData data;
  std::string base_folder;

 public:
  Config(std::string cfg_fname);
  // Calibration folder full path (base + calib_folder)
  const std::string calib_full();
  // shader folder full path (base + shader_folder)
  const std::string shader_full();
  const std::string cam_pose_filename();
  const std::string image_filenames(int i);
  const std::string calib_filenames(int i);
  const std::string robot_model_filename();

  const std::string lrf_data_filename();
  const std::string LRF_com_port();

  const std::string shader_folder();
  const int num_camera();
  const std::vector<std::string> image_sources();
  const int capture_framerate();

  void getRobotPose(cv::Mat &trans_matrix);

  void getLRFPose(double &trans_x, double &trans_y, double &rot_rad);
};

}  // namespace fvp
