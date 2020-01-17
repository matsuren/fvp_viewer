#include <iostream>
#include <fstream>
#include <string>

#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>


struct SettingParameters {
	// data folder that contains calibrated data.
	// ex. img*.jpg, calib_results_*.txt, final_camera_poses.yml
	std::string calib_folder = "../../data";
	std::string record_folder = "../../raw_data";
	std::vector<std::string> image_sources = { "15637060", "15637085","16025862","16025863" };

	int capture_framerate = 20;
	// LRF COM port
	std::string LRF_com_port = "COM4";
private:
	std::string ocamcalib_files = "calib_results_*.txt";
	std::string sample_image_files = "img*.jpg";
	std::string camera_pose_yml = "final_camera_poses.yml";
	std::string robot_model_file = "robot.ply";
	std::string robot_align_yml = "robot_align_matrix.yml";
	std::string lrf_align_yml = "lrf_align_matrix.yml";
	std::string sample_lrf_file = "urg_xy.csv";


public:
	// 
	SettingParameters()
	{
	}

	std::string cam_poses() {
		return calib_folder + "/" + camera_pose_yml;
	}
	std::string imagefile(int i) {
		// copy
		std::string s = sample_image_files;
		// replace
		s.replace(s.find("*"), 1, std::to_string(i));
		return calib_folder + "/" + s;
	}
	std::string calibfile(int i) {
		// copy
		std::string s = ocamcalib_files;
		// replace
		s.replace(s.find("*"), 1, std::to_string(i));
		return calib_folder + "/" + s;
	}
	std::string robot_model() {
		return calib_folder + "/" + robot_model_file;
	}

	std::string lrf_file() {
		return calib_folder + "/" + sample_lrf_file;
	}

	void robot_align(cv::Mat &trans_matrix) {
		const auto fs = cv::FileStorage(calib_folder + "/" + robot_align_yml, cv::FileStorage::READ);
		if (!fs.isOpened()) {
			std::cout << "cann't open file:" << robot_align_yml << " in " << calib_folder << std::endl;
			return;
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

	void lrf_align(double &trans_x, double &trans_y, double &rot_rad) {
		const auto fs = cv::FileStorage(calib_folder + "/" + lrf_align_yml, cv::FileStorage::READ);
		if (!fs.isOpened()) {
			std::cout << "cann't open file:" << lrf_align_yml << " in " << calib_folder << std::endl;
			return;
		}
		cv::Mat tvec;
		fs["lrf_rot"] >> rot_rad;
		fs["lrf_tvec"] >> tvec;
		trans_x = tvec.at<double>(0, 0);
		trans_y = tvec.at<double>(1, 0);
	}

	template<class Archive>
	void serialize(Archive & archive)
	{
		archive(CEREAL_NVP(calib_folder), CEREAL_NVP(record_folder), CEREAL_NVP(image_sources),
			CEREAL_NVP(ocamcalib_files), CEREAL_NVP(sample_image_files), CEREAL_NVP(camera_pose_yml),
			CEREAL_NVP(robot_model_file), CEREAL_NVP(robot_align_yml),
			CEREAL_NVP(capture_framerate), 
			CEREAL_NVP(LRF_com_port), CEREAL_NVP(lrf_align_yml), CEREAL_NVP(sample_lrf_file));
	}
};
