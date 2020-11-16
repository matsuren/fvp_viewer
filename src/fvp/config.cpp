#include "fvp/config.hpp"

#include <spdlog/spdlog.h>

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <string>

namespace fvp {

ConfigData::ConfigData()
    : calib_folder("data"),
      shader_folder("shader"),
      image_sources({"15637060", "15637085", "16025862", "16025863"}),
      capture_framerate(20),
      LRF_com_port("COM4"),
      ocamcalib_files("calib_results_*.txt"),
      sample_image_files("img*.jpg"),
      camera_pose_yml("final_camera_poses.yml"),
      robot_model_file("robot.ply"),
      robot_align_yml("robot_align_matrix.yml"),
      lrf_align_yml("lrf_align_matrix.yml"),
      sample_lrf_file("urg_xy.csv") {}

template <class Archive>
void ConfigData::serialize(Archive &archive) {
  archive(CEREAL_NVP(calib_folder), CEREAL_NVP(shader_folder),
          CEREAL_NVP(image_sources), CEREAL_NVP(ocamcalib_files),
          CEREAL_NVP(sample_image_files), CEREAL_NVP(camera_pose_yml),
          CEREAL_NVP(robot_model_file), CEREAL_NVP(robot_align_yml),
          CEREAL_NVP(capture_framerate), CEREAL_NVP(LRF_com_port),
          CEREAL_NVP(lrf_align_yml), CEREAL_NVP(sample_lrf_file));
}

Config::Config(std::string cfg_fname) {
  // Load config file
  // If there is no config file, it will generate default config file
  spdlog::info("Load config file: {}", cfg_fname);
  const size_t found = cfg_fname.find_last_of("/\\");
  std::string fname;
  if (found != std::string::npos) {
    base_folder = cfg_fname.substr(0, found);
    fname = cfg_fname.substr(found + 1);
  } else {
    base_folder = ".";
    fname = cfg_fname;
  }
  spdlog::info("Base folder of config file: {}", base_folder);

  std::ifstream ifs(cfg_fname);
  try {
    cereal::JSONInputArchive i_archive(ifs);
    i_archive(data);
  } catch (cereal::Exception &e) {
    if (!ifs) {
      spdlog::error("Cannot read config file : {}", fname);
    } else {
      spdlog::error("cereal::Exception : {}", e.what());
    }
    std::ofstream os(fname);
    spdlog::info("Generating default config file : ./{}", fname);
    cereal::JSONOutputArchive o_archive(os);
    o_archive(cereal::make_nvp("FVP_settings", data));
    throw std::runtime_error("No config or wrong config file");
  }
}
const std::string Config::calib_full() {
  return fmt::format("{}/{}", base_folder, data.calib_folder);
}
const std::string Config::shader_full() {
  return fmt::format("{}/{}", base_folder, data.shader_folder);
}
const std::string Config::cam_pose_filename() {
  return fmt::format("{}/{}", calib_full(), data.camera_pose_yml);
}
const std::string Config::image_filenames(int i) {
  // copy
  std::string s = data.sample_image_files;
  // replace
  s.replace(s.find("*"), 1, std::to_string(i));
  return fmt::format("{}/{}", calib_full(), s);
}
const std::string Config::calib_filenames(int i) {
  // copy
  std::string s = data.ocamcalib_files;
  // replace
  s.replace(s.find("*"), 1, std::to_string(i));
  return fmt::format("{}/{}", calib_full(), s);
}
const std::string Config::robot_model_filename() {
  return fmt::format("{}/{}", calib_full(), data.robot_model_file);
}

const std::string Config::lrf_data_filename() {
  return fmt::format("{}/{}", calib_full(), data.sample_lrf_file);
}

const std::string Config::LRF_com_port() { return data.LRF_com_port; }

const std::string Config::shader_folder() { return data.shader_folder; }
const int Config::num_camera() { return int(data.image_sources.size()); }
const std::vector<std::string> Config::image_sources() {
  return data.image_sources;
}
const int Config::capture_framerate() { return data.capture_framerate; }

void Config::getRobotPose(cv::Mat &trans_matrix) {
  const auto fs =
      cv::FileStorage(fmt::format("{}/{}", calib_full(), data.robot_align_yml),
                      cv::FileStorage::READ);
  if (!fs.isOpened()) {
    spdlog::error("Cannot open file: {} in {}.", data.robot_align_yml,
                  calib_full());
    throw std::runtime_error("No robot pose file");
  }
  cv::Mat R, rvec, tvec;
  double scale;
  fs["robot_scale"] >> scale;
  fs["robot_rvec"] >> rvec;
  fs["robot_tvec"] >> tvec;
  trans_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);
  cv::Rodrigues(rvec, R);
  R *= scale;
  R.convertTo(trans_matrix(cv::Rect(0, 0, 3, 3)), CV_32F);
  tvec.convertTo(trans_matrix(cv::Rect(3, 0, 1, 3)), CV_32F);
}

void Config::getLRFPose(double &trans_x, double &trans_y, double &rot_rad) {
  const auto fs =
      cv::FileStorage(fmt::format("{}/{}", calib_full(), data.lrf_align_yml),
                      cv::FileStorage::READ);

  if (!fs.isOpened()) {
    spdlog::error("Cannot open file: {} in {}.", data.lrf_align_yml,
                  calib_full());
    throw std::runtime_error("No LRF pose file");
  }
  cv::Mat tvec;
  fs["lrf_rot"] >> rot_rad;
  fs["lrf_tvec"] >> tvec;
  trans_x = tvec.at<double>(0, 0);
  trans_y = tvec.at<double>(1, 0);
}
}  // namespace fvp
