#pragma once
#include <spdlog/spdlog.h>

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>
#include <fstream>
#include <iostream>
#include <string>

namespace fvp {
struct ConfigData {
 public:
  // data folder that contains calibrated data.
  // ex. img*.jpg, calib_results_*.txt, final_camera_poses.yml
  std::string calib_folder;
  std::string record_folder;
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

  //
  ConfigData()
      : calib_folder("../../data"),
        record_folder("../../raw_data"),
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
  void serialize(Archive &archive) {
    archive(CEREAL_NVP(calib_folder), CEREAL_NVP(record_folder),
            CEREAL_NVP(image_sources), CEREAL_NVP(ocamcalib_files),
            CEREAL_NVP(sample_image_files), CEREAL_NVP(camera_pose_yml),
            CEREAL_NVP(robot_model_file), CEREAL_NVP(robot_align_yml),
            CEREAL_NVP(capture_framerate), CEREAL_NVP(LRF_com_port),
            CEREAL_NVP(lrf_align_yml), CEREAL_NVP(sample_lrf_file));
  }
};

class Config {
 private:
  ConfigData data;

 public:
  Config(std::string cfg_fname) {
    // Load config file
    // If there is no config file, it will generate default config file
    spdlog::info("Load config file: {}", cfg_fname);
    std::ifstream ifs(cfg_fname);
    if (!ifs) {
      spdlog::error("Cannot read config file : {}", cfg_fname);
      spdlog::info("Generating default config file in {}", cfg_fname);
      std::ofstream os(cfg_fname);
      {
        cereal::JSONOutputArchive o_archive(os);
        o_archive(cereal::make_nvp("FVP_settings", data));
      }
      throw std::runtime_error("No config file");
    }
    cereal::JSONInputArchive i_archive(ifs);
    i_archive(data);
  }

  std::string cam_pose_filename() {
    return data.calib_folder + "/" + data.camera_pose_yml;
  }
  std::string image_filenames(int i) {
    // copy
    std::string s = data.sample_image_files;
    // replace
    s.replace(s.find("*"), 1, std::to_string(i));
    return data.calib_folder + "/" + s;
  }
  std::string calib_filenames(int i) {
    // copy
    std::string s = data.ocamcalib_files;
    // replace
    s.replace(s.find("*"), 1, std::to_string(i));
    return data.calib_folder + "/" + s;
  }
  std::string robot_model_filename() {
    return data.calib_folder + "/" + data.robot_model_file;
  }

  std::string lrf_data_filename() {
    return data.calib_folder + "/" + data.sample_lrf_file;
  }

  std::string LRF_com_port() { return data.LRF_com_port; }

  std::string record_folder() { return data.record_folder; }
  size_t num_camera() { return data.image_sources.size(); }
  std::vector<std::string> image_sources() { return data.image_sources; }
  int capture_framerate() { return data.capture_framerate; }

  void getRobotPose(cv::Mat &trans_matrix) {
    const auto fs = cv::FileStorage(
        data.calib_folder + "/" + data.robot_align_yml, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      spdlog::error("Cannot open file: {} in {}.", data.robot_align_yml,
                    data.calib_folder);
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

  void getLRFPose(double &trans_x, double &trans_y, double &rot_rad) {
    const auto fs = cv::FileStorage(
        data.calib_folder + "/" + data.lrf_align_yml, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      spdlog::error("Cannot open file: {} in {}.", data.lrf_align_yml,
                    data.calib_folder);
      throw std::runtime_error("No LRF pose file");
    }
    cv::Mat tvec;
    fs["lrf_rot"] >> rot_rad;
    fs["lrf_tvec"] >> tvec;
    trans_x = tvec.at<double>(0, 0);
    trans_y = tvec.at<double>(1, 0);
  }
};

}  // namespace fvp
