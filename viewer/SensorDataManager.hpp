#include <iostream>
#include <fstream>
#include <array>
#include <mutex>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <cereal/archives/json.hpp>

#include "cameras/spinmanager.hpp"
#include "cameras/spincamera.hpp"
#include "cameras/RecordImageManager.hpp"
#include "utils/FpsDisplayer.hpp"
#include "utils/SettingParameters.hpp"
#include "glslcookbook/cookbookogl.h"
#include "LRFSensor.hpp"
#include "main.hpp"


// singleton SensorDataManager CLASS
class SensorDataManager {
private:
	// private constractor
	SensorDataManager()
	{
		std::cout << "initialize SensorDataManager : " << std::endl;
		manager = std::make_unique<SpinManager>();
		initialize();
	}
	~SensorDataManager() {
	};
public:
	SensorDataManager(const SensorDataManager&) = delete;
	SensorDataManager& operator=(const SensorDataManager&) = delete;
	SensorDataManager(SensorDataManager&&) = delete;
	SensorDataManager& operator=(SensorDataManager&&) = delete;
	// -----------------------------------
	static SensorDataManager& getInstance() {
		static SensorDataManager inst;
		return inst;
	}

	// -----------------------------------
	// get camera image
	int getCameraImage(cv::Mat &ret_mat, const int camera_id)
	{
		if (static_cast<size_t>(camera_id) < capture_imgs.size())
		{
			ret_mat = capture_imgs[camera_id];
			return 0;
		}
		else
		{
			std::cout << "error! capture_imgs[" << camera_id << "] is nothing!" << std::endl;
			return -1;
		}
	}
	// -----------------------------------
	// get camera image
	int getCameraNumber()
	{
		return static_cast<int>(capture_imgs.size());
	}
	// -----------------------------------
	// get camera image
	int setPixelbuffer(const GLuint pixel_buffer, const int camera_id)
	{
		if (static_cast<size_t>(camera_id) < capture_imgs.size())
		{
			img_pixel_buffers[camera_id] = pixel_buffer;
			return 0;
		}
		else
		{
			std::cout << "error! img_pixel_buffers[" << camera_id << "] is nothing!" << std::endl;
			return -1;
		}
	}
	// -----------------------------------
	// get camera image
	GLuint getPixelbuffer(const int camera_id)
	{
		if (static_cast<size_t>(camera_id) < capture_imgs.size())
		{
			return img_pixel_buffers[camera_id];
		}
		else
		{
			std::cout << "error! img_pixel_buffers[" << camera_id << "] is nothing!" << std::endl;
			return -1;
		}
	}
	// -----------------------------------
	// transposed (the source array is stored column-wise)
	int getCameraViewMatrix(cv::Mat &ret_mat, const int camera_id)
	{
		if (static_cast<size_t>(camera_id) < fisheye_views.size())
		{
			cv::transpose(fisheye_views[camera_id], ret_mat);
			return 0;
		}
		else
		{
			std::cout << "error! capture_imgs[" << camera_id << "] is nothing!" << std::endl;
			return -1;
		}
	}
	// -----------------------------------
	int setCameraViewMatrix(const cv::Mat& Rt, const int camera_id)
	{
		if (static_cast<size_t>(camera_id) < fisheye_views.size())
		{
			Rt.convertTo(fisheye_views[camera_id], CV_32F);
		}
		else
		{
			std::cout << "setCameraViewMatrix error!\n capture_imgs[" << camera_id << "] is nothing!" << std::endl;
			return -1;
		}
		return 0;
	}
	// -----------------------------------

	// update
	void update(float t)
	{
		// texture
		for (size_t i = 0; i < capture_imgs.size(); ++i) {
			//if (i < cams.size() ) {
			if (i < cams.size() && processed_srcs_is_new[i]) {

				glBindBuffer(GL_PIXEL_UNPACK_BUFFER, img_pixel_buffers[i]);

				// map the buffer object into client's memory
				// Note that glMapBufferARB() causes sync issue.
				// If GPU is working with this buffer, glMapBufferARB() will wait(stall)
				// for GPU to finish its job. To avoid waiting (stall), you can call
				// first glBufferDataARB() with NULL pointer before glMapBufferARB().
				// If you do that, the previous data in PBO will be discarded and
				// glMapBufferARB() returns a new allocated pointer immediately
				// even if GPU is still working with the previous data.
				int SRC_DATA_SIZE = processed_srcs[i].cols * processed_srcs[i].rows * 3;
				glBufferData(GL_PIXEL_UNPACK_BUFFER, SRC_DATA_SIZE, 0, GL_DYNAMIC_DRAW);
				GLubyte* ptr = (GLubyte*)glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
				if (ptr)
				{
					{
						std::lock_guard<std::mutex> lock(*mtxs[i]);
						// update data directly on the mapped buffer
						//updatePixels(ptr, DATA_SIZE);
						memcpy(ptr, processed_srcs[i].data, SRC_DATA_SIZE);
					}
					glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER); // release pointer to mapping buffer
				}
				glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, GLint(i), processed_srcs[i].cols, processed_srcs[i].rows, 1, GL_BGR, GL_UNSIGNED_BYTE, nullptr);
				processed_srcs_is_new[i] = false;
			}
		}
		// LRF Data update
		//LRF_data_is_new = true;
		if (LRF_data_is_new)
		{
			std::vector<float> vertices;
			std::vector<GLuint> elements;

			std::lock_guard<std::mutex> lock(LRF_mtx);
			getLRFGLdata(LRF_data, vertices, elements);
			LRF_vertices_num = int(vertices.size());
			glBindVertexArray(LRF_vaoHandle);

			glBindBuffer(GL_ARRAY_BUFFER, LRF_handle[0]);
			glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_DYNAMIC_DRAW);
			glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte *)NULL + (0)));
			glEnableVertexAttribArray(0);  // Vertex position

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, LRF_handle[1]);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, elements.size() * sizeof(GLuint), &elements[0], GL_DYNAMIC_DRAW);

			glBindVertexArray(0);

			LRF_data_is_new = false;
		}
	}


	//-----------------------------------------------------------------------------
	void drawModel(const std::string model_name) {
		if (model_name == "LRF")
		{
			glBindVertexArray(LRF_vaoHandle);
			glDrawElements(GL_TRIANGLES, LRF_vertices_num, GL_UNSIGNED_INT, ((GLubyte *)NULL + (0)));
		}
	}


	//-----------------------------------------------------------------------------
	void join() {
		for (auto &it : ths) {
			it.join();
		}
		ths.clear();
	}

public:
	// setting parameters
	SettingParameters setting_params;

private:
	std::unique_ptr<SpinManager> manager;
	// camera image
	std::vector<cv::Mat> capture_imgs;
	std::vector<GLuint> img_pixel_buffers;
	std::vector<cv::Mat> fisheye_views;

	std::vector<SpinCamPtr> cams;
	std::vector<cv::Mat> raw_srcs;
	std::vector<cv::Mat> processed_srcs;
	std::vector<bool> processed_srcs_is_new;
	std::vector<std::mutex*> mtxs;

	// LRF data
	LRFSensor *LRF_sensor;
	std::vector<LRFPoint> LRF_data;
	float LRF_model_height = 3.0f;
	int LRF_vertices_num = 0;
	unsigned int LRF_vaoHandle;
	unsigned int LRF_handle[2];
	std::mutex LRF_mtx;
	bool LRF_data_is_new;

	// thread 
	std::vector<std::thread> ths;

	// record raw folder
	std::string record_foldername;


	// -----------------------------------
	int initialize()
	{
		//////////////////////////
		// load setting paramters
		//////////////////////////
		{
			std::string fname = "../../config_FVP_parameters.json";
			std::ifstream ifs(fname);
			if (!ifs)
			{
				std::cout << "// ------------------------------------//\n";
				std::cout << "Cannot read config file : " << fname << std::endl;;
				std::cout << "Generating default config file..." << std::endl;
				std::ofstream os(fname);
				{
					cereal::JSONOutputArchive o_archive(os);
					o_archive(cereal::make_nvp("FVP_settings", setting_params));
				}
				std::cout << "output : " << fname << std::endl;
				std::cout << "// ------------------------------------//\n";

				exit(-1);
			}
			cereal::JSONInputArchive i_archive(ifs);
			i_archive(setting_params);
		}


		//////////////////////////
		// load camera images
		//////////////////////////
		for (int i = 0; i < 4; i++)
		{
			const std::string fname = setting_params.imagefile(i);
			cv::Mat tmp = cv::imread(fname);
			if (tmp.empty())
			{
				std::cout << "cannot load image : " << fname << std::endl;
				return -1;
			}
			capture_imgs.push_back(tmp);
			img_pixel_buffers.push_back(0);
			fisheye_views.push_back(cv::Mat::eye(4, 4, CV_32FC1));
		}


		//////////////////////////
		// add camera
		//////////////////////////

		std::vector<std::string> image_sources = setting_params.image_sources;

		//std::string raw_recorded_folder = "../../raw_data";
		//std::vector<std::string> image_sources = { raw_recorded_folder + "/0_*.pgm" ,raw_recorded_folder + "/1_*.pgm",
		//	raw_recorded_folder + "/2_*.pgm" , raw_recorded_folder + "/3_*.pgm" };

		// add camera
		for (size_t i = 0; i < image_sources.size(); ++i) {
			std::map<std::string, std::string> values;
			values["framerate"] = std::to_string(setting_params.capture_framerate);
			addCamera(image_sources[i], values);
		}

		// Recoder folder
		RecordImageManager::getInstance().setRecordFolder(setting_params.record_folder, setting_params.capture_framerate);

		//////////////////////////
		// load LRF data
		//////////////////////////
		std::ifstream ifs_lrf(setting_params.lrf_file());
		if (!ifs_lrf) {
			std::cout << "error !\n cannot read LRF data file : " << setting_params.lrf_file() << std::endl;
			return 1;
		}
		// read from csv file
		LRF_data.clear();
		std::string str;
		while (getline(ifs_lrf, str)) {
			std::vector<std::string> ret_str = split(str, ",");
			LRFPoint tmp_pair(std::stof(ret_str[0]), std::stof(ret_str[1]));
			LRF_data.push_back(tmp_pair);
		}

		std::vector<float> vertices;
		std::vector<GLuint> elements;

		getLRFGLdata(LRF_data, vertices, elements);

		LRF_vertices_num = int(vertices.size());
		glGenVertexArrays(1, &LRF_vaoHandle);
		glBindVertexArray(LRF_vaoHandle);
		glGenBuffers(2, LRF_handle);

		glBindBuffer(GL_ARRAY_BUFFER, LRF_handle[0]);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_DYNAMIC_DRAW);
		glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte *)NULL + (0)));
		glEnableVertexAttribArray(0);  // Vertex position

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, LRF_handle[1]);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, elements.size() * sizeof(GLuint), &elements[0], GL_DYNAMIC_DRAW);

		glBindVertexArray(0);
		LRF_data_is_new = false;
		if (g_argc == 3)
		{
			LRF_sensor = new LRFSensor(g_argc, g_argv);
			std::cout << "**** LRF port : " << g_argv[2] << std::endl;
		}
		else
		{
			std::string str = setting_params.LRF_com_port;
			char * writable = new char[str.size() + 1];
			std::copy(str.begin(), str.end(), writable);
			writable[str.size()] = '\0'; // don't forget the terminating 0
			char *argv[] = { "exe", "-s", writable };
			LRF_sensor = new LRFSensor(3, argv);
			std::cout << "**** LRF port : " << argv[2] << std::endl;

			// don't forget to free the string after finished using it						
			delete[] writable;
		}

		record_foldername = setting_params.record_folder;
		ths.push_back(std::thread(&SensorDataManager::LRFWorker, this, true));

		return 0;
	}

	//-----------------------------------------------------------------------------
	void addCamera(const std::string &image_source, std::map<std::string, std::string> &values) {
		SpinCamPtr cam;
		// Spinnaker Camera (serial number)
		if (image_source.size() == 8) {
			if (manager->serial2idx.find(image_source) != manager->serial2idx.end())
				cam = std::make_shared<SpinCam>(manager->getCamera(image_source));
		}
		// Spinnaker Camera
		else {
			int camera_num = std::stoi(image_source);
			if (camera_num <= manager->size())
				cam = std::make_shared<SpinCam>(manager->getCamera(camera_num));
		}
		// set fps : Display freshrate
		if (values.find("framerate") != values.end()) {
			double fps = std::stod(values["framerate"]);
			if(cam)
				cam->setFrameRate(fps);
		}
		cams.push_back(cam);

		//// White balance
		//cams.back()->setWhiteBalanceRatio(1.18, "Red");
		//cams.back()->setWhiteBalanceRatio(1.46, "Blue");

		// initialize container
		raw_srcs.push_back(cv::Mat());
		processed_srcs.push_back(capture_imgs[cams.size() - 1]);
		processed_srcs_is_new.push_back(false);
		mtxs.push_back(new std::mutex());
	}
public:
	//-----------------------------------------------------------------------------
	void startCapture(bool useImshow) {
		size_t CAMERA_NUM = cams.size();
		//
		bool fps_flag = true;
		for (size_t i = 0; i < CAMERA_NUM; ++i) {
			// display fps only one
			if (cams[i]) {
				ths.push_back(std::thread(&SensorDataManager::captureWorker, this, i, fps_flag));
				fps_flag = false;
			}
		}
		if (useImshow)
			ths.push_back(std::thread(&SensorDataManager::viewerWorker, this));
	}


	//-----------------------------------------------------------------------------
	void captureWorker(int cam_id, bool enableFPS) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		// get fps
		FpsDisplayer fps_displayer("capture image " + std::to_string(cam_id), 160);
		if (enableFPS)
			fps_displayer.start();

		cv::Size img_size = processed_srcs[cam_id].size();
		// malfunction image
		cv::Mat malfunction_mat = cv::Mat::zeros(processed_srcs[cam_id].size(), CV_8UC3);
		cv::line(malfunction_mat, cv::Point(0, 0), cv::Point(malfunction_mat.cols, malfunction_mat.rows), cv::Scalar(0, 0, 200), malfunction_mat.cols / 20);
		cv::line(malfunction_mat, cv::Point(malfunction_mat.cols, 0), cv::Point(0, malfunction_mat.rows), cv::Scalar(0, 0, 200), malfunction_mat.cols / 20);

		std::vector<std::thread> capture_ths;
		std::vector<cv::Mat> capture_image;

		try
		{
			// loop
			while (!checkExit()) {
				if (cams[cam_id]->read(raw_srcs[cam_id], false))
				{
					{
						std::lock_guard<std::mutex> lock(*mtxs[cam_id]);
						cv::cvtColor(raw_srcs[cam_id], processed_srcs[cam_id], cv::COLOR_BayerGR2BGR);
						if (img_size != processed_srcs[cam_id].size()) {
							// use image
							std::cout << "\nimage size in sample folder: " << img_size << " vs camera image: " << processed_srcs[cam_id].size() << "\n";
							std::cout << "\n/*********** camera image size doesn't match !!! Use image in sample folder. ***********\n";
							std::this_thread::sleep_for(std::chrono::milliseconds(33));
							break;
						}
						processed_srcs_is_new[cam_id] = true;
					}
					// for capture
					if (RecordImageManager::getInstance().isRecorded())
					{
						capture_image.push_back(raw_srcs[cam_id].clone());
					}
				}
				else
				{
					if (raw_srcs[cam_id].empty())
					{
						// use image
						std::cout << "\n/*********** camera " << cam_id << " is not working!  use image. ***********\n";
						std::this_thread::sleep_for(std::chrono::milliseconds(33));
						break;
					}
					else
					{
						// camera malfunction
						processed_srcs[cam_id] = malfunction_mat;
						processed_srcs_is_new[cam_id] = true;
					}
				}
				// display fps
				if (enableFPS)
					fps_displayer.addCount();
			}
		}
		catch (const std::exception& e)
		{
			std::cout << e.what();
			threadExit();
		}
		if (RecordImageManager::getInstance().already_create_directory)
		{
			std::string RAW_FOLDER = RecordImageManager::getInstance().getRecordFolder() + "/";
			std::this_thread::sleep_for(std::chrono::milliseconds(1000 * cam_id));
			std::cout << cam_id << "record start!\n";
			for (size_t i = 0; i < capture_image.size(); ++i)
			{
				std::string filename = RAW_FOLDER + std::to_string(cam_id) + "_" + std::to_string(i) + ".pgm";
				capture_ths.push_back(std::thread([](std::string fname, cv::Mat &img) {
					if (!cv::imwrite(fname, img))
					{
						std::cout << fname << " : imwrite error!";
					} }
				, filename, capture_image[i]));

				// pause
				if (i % 5 == 0)
				{
					for (auto &it : capture_ths)
						it.join();
					capture_ths.clear();
				}
			}
			for (auto &it : capture_ths)
				it.join();

			std::cout << cam_id << " record end!\n";
		}
	}

	//-----------------------------------------------------------------------------
	void viewerWorker() {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		// get fps
		FpsDisplayer fps_displayer("viewer", 250);
		fps_displayer.start();
		while (!checkExit()) {
			size_t CAMERA_NUM = cams.size();
			for (size_t i = 0; i < CAMERA_NUM; ++i) {
				if (!processed_srcs[i].empty()) {
					std::lock_guard<std::mutex> lock(*mtxs[i]);
					cv::namedWindow(std::to_string(i), cv::WINDOW_NORMAL);
					cv::imshow(std::to_string(i), processed_srcs[i]);
				}
				else {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
				int key = cv::waitKey(5);
				if (key == 27) threadExit();
			}
			// display fps
			fps_displayer.addCount();
		}
	}
	//-----------------------------------------------------------------------------
	void LRFWorker(bool enableFPS) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		// get fps
		FpsDisplayer fps_displayer("LRF data ", 160);
		if (enableFPS)
			fps_displayer.start();

		//for capture
		const std::string RAW_FOLDER = record_foldername + "/";
		bool capture_flag;
		{
			std::string tmp_fname = RAW_FOLDER + "urg_.csv";
			std::ofstream ofs(tmp_fname);
			capture_flag = ofs.is_open();
		}
		if (capture_flag)
			std::cout << "**** LRF save raw data ON *******\n";
		else
			std::cout << "**** LRF save raw data OFF *******\n";
		std::vector<std::thread> capture_ths;
		std::vector<std::vector<LRFPoint>> capture_LRF_data;
		int capture_count = 0;

		// loop
		while (!checkExit()) {
			if (LRF_sensor->grab())
			{
				std::lock_guard<std::mutex> lock(LRF_mtx);
				LRF_sensor->retrieve(LRF_data);
				LRF_data_is_new = true;

				// for capture
				if (capture_flag)
				{
					capture_LRF_data.push_back(LRF_data);
				}
			}
			else
			{
				// use image
				std::cout << "\n/*********** LRF  is not working!  use saved data. ***********\n";
				std::this_thread::sleep_for(std::chrono::milliseconds(33));
				break;
			}
			// display fps
			if (enableFPS)
				fps_displayer.addCount();
		}

		if (capture_flag)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			std::cout << "LRF record start!\n";
			for (size_t i = 0; i < capture_LRF_data.size(); ++i)
			{
				std::string filename = RAW_FOLDER + "urg_xy_" + std::to_string(i) + ".csv";
				capture_ths.push_back(std::thread([](std::string fname, std::vector<LRFPoint> data) {
					std::ofstream ofs(fname);
					for (const auto &it : data) {
						ofs << it.x << ", " << it.y << std::endl;
					}
				}
				, filename, capture_LRF_data[i]));

				// pause
				if (i % 10 == 0)
				{
					for (auto &it : capture_ths)
						it.join();
					capture_ths.clear();
				}
			}
			for (auto &it : capture_ths)
				it.join();

			std::cout << "LRF record end!\n";
		}
	}

	//-----------------------------------------------------------------------------
	std::vector<std::string> split(const std::string& s, const std::string delim)
	{
		std::vector<std::string> result;
		result.clear();

		using string = std::string;
		string::size_type pos = 0;

		while (pos != string::npos)
		{
			string::size_type p = s.find(delim, pos);

			if (p == string::npos)
			{
				result.push_back(s.substr(pos));
				break;
			}
			else {
				result.push_back(s.substr(pos, p - pos));
			}

			pos = p + delim.size();
		}

		// compress
		for (size_t i = 0; i < result.size(); i++) {
			if (result[i] == "" || result[i] == delim) {
				result.erase(result.begin() + i);
				i--;
			}
		}

		return result;
	}
	//-----------------------------------------------------------------------------
	void getLRFGLdata(const std::vector<LRFPoint> &LRF_data,
		std::vector<float> &vertices, std::vector<GLuint> &elements)
	{
		const double distance_threshold = 30.0;
		vertices.clear();
		vertices.reserve(3 * 4 * 1200);
		elements.clear();
		elements.reserve(3 * 3 * 1200);

		// origin
		vertices.push_back(0.0f);
		vertices.push_back(0.0f);
		vertices.push_back(0.0f);

		for (size_t i = 1; i < LRF_data.size(); i++)
		{
			double diff_x = LRF_data[i].x - LRF_data[i - 1].x;
			double diff_y = LRF_data[i].y - LRF_data[i - 1].y;
			double distance = sqrt(diff_x * diff_x + diff_y * diff_y);

			int current_num = int(vertices.size() / 3);
			// vertex
			vertices.push_back(LRF_data[i - 1].x);
			vertices.push_back(LRF_data[i - 1].y);
			vertices.push_back(0.0f);

			vertices.push_back(LRF_data[i - 1].x);
			vertices.push_back(LRF_data[i - 1].y);
			vertices.push_back(LRF_model_height);

			vertices.push_back(LRF_data[i].x);
			vertices.push_back(LRF_data[i].y);
			vertices.push_back(LRF_model_height);

			vertices.push_back(LRF_data[i].x);
			vertices.push_back(LRF_data[i].y);
			vertices.push_back(0.0f);

			if (distance < distance_threshold)
			{
				// element
				elements.push_back(current_num);
				elements.push_back(current_num + 1);
				elements.push_back(current_num + 2);

				elements.push_back(current_num);
				elements.push_back(current_num + 2);
				elements.push_back(current_num + 3);
			}

			// floor
			elements.push_back(0);
			elements.push_back(current_num);
			elements.push_back(current_num + 3);

		}

		// floor loop close
		elements.push_back(0);
		int current_num = int(vertices.size() / 3) - 1;
		elements.push_back(current_num);
		elements.push_back(1);
	}
	//------------------------------------------------------------------------------

};
