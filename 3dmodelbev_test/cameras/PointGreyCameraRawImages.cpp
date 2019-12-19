
/*-------------
// 一台のPointGreyCameraRawImagesで得られたraw画像を用いるクラス
----------------*/

#include <iostream>
#include <opencv2\core\core.hpp>
#include "PointGreyCameraRawImages.hpp"
#include <opencv2/imgcodecs.hpp>
#include "LockedQueue.hpp"


//-----------------------------------------------------------------------------
PointGreyCameraRawImages::PointGreyCameraRawImages(const std::string raw_fname, std::map<std::string, std::string> &values)
{
	locked_queue = new LockedQueue<cv::Mat>(MAX_QUEUE_SIZE);

	// raw serial images
	auto pos = raw_fname.find("*");
	if(pos != std::string::npos)
	{
		raw_fname_first = raw_fname.substr(0, pos);
		raw_fname_latter = raw_fname.substr(pos+1);
	}
	initializeCapture(values);
	captureFlag_ = true;
	ths.push_back(std::thread(&PointGreyCameraRawImages::queueWorker, this));

	// check if buffer is full
	while(locked_queue->getCurrentDataSize()!= MAX_QUEUE_SIZE && captureFlag_)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
	}
}

//-----------------------------------------------------------------------------
void PointGreyCameraRawImages::initializeCapture(std::map<std::string, std::string> &values) {
	if (values.find("FlyCapture2::Mode") == values.end()) {
		values["FlyCapture2::Mode"] = "Mode_0";
	}
	// Set the property
	SetProperty(values);


}
//-----------------------------------------------------------------------------
PointGreyCameraRawImages::~PointGreyCameraRawImages(void)
{
	stopCapture();
}
// カメラの情報を表示
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
bool PointGreyCameraRawImages::SetProperty(const std::map<std::string, std::string> &values)
{

	// set ROI
	if (values.find("FlyCapture2::offsetX") != values.end()) {
		offsetX = std::stoi(values.at(std::string("FlyCapture2::offsetX")));
	}
	if (values.find("FlyCapture2::offsetY") != values.end()) {
		offsetY = std::stoi(values.at(std::string("FlyCapture2::offsetY")));
	}
	if (values.find("FlyCapture2::width") != values.end()) {
		// width must be 32 * n
		int n = std::stoi(values.at(std::string("FlyCapture2::width"))) / 32;
		width = n * 32;
	}
	if (values.find("FlyCapture2::height") != values.end()) {
		// height must be 2 * n
		int n = std::stoi(values.at(std::string("FlyCapture2::height"))) / 2;
		height = n * 2;
	}

	// set original roi
	if (width == -1 || height == -1 || offsetX == -1 || offsetY == -1)
	{
		std::cout << "use original size\n";
	}else
	{
		std::cout << "image offset : (" << offsetX << ", " << offsetY << ")   ";
		std::cout << "image size : (" << width << ", " << height << ") \n";
	}


	return false;
}



//-----------------------------------------------------------------------------
int PointGreyCameraRawImages::grabRaw(void)
{
	if (!captureFlag_) {
		std::cout << "capture error" << std::endl;
		return 0;
	}

	static auto t0 = std::chrono::high_resolution_clock::now();
	// image load
	frame_ = locked_queue->dequeue();
	auto t1 = std::chrono::high_resolution_clock::now();
	// adjust video fps
	auto wait_time = long long(1.0 / fps * 1000000) - std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
	std::this_thread::sleep_for(
		std::chrono::microseconds((wait_time > 0) ? wait_time : 0));

	// save time 
	t0 = std::chrono::high_resolution_clock::now();

	return 1;
}
//-----------------------------------------------------------------------------
int PointGreyCameraRawImages::grab(void)
{


	return 1;
}
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
bool PointGreyCameraRawImages::retrieve(cv::Mat& image)
{
	image = frame_;
	return true;
}
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
void PointGreyCameraRawImages::queueWorker() {

	// 
	bool need_crop;

	// folder check
	{
		// image load
		std::lock_guard<std::mutex> lock(mtx);
		std::string filename = raw_fname_first + std::to_string(0) + raw_fname_latter;
		cv::Mat frame;
		frame = cv::imread(filename, cv::IMREAD_GRAYSCALE);
		if (frame.empty()) {
			std::cout << "/***************/\n" << "error! " << filename << " does not exist!" << "\n/***************/\n";
			captureFlag_ = false;
		}

		// if roi is not set, use original image
		if(width == -1|| height == -1 || offsetX == -1 || offsetY == -1)
		{
			width = frame.cols;
			height = frame.rows;
			offsetX = 0;
			offsetY = 0;
			need_crop = false;
		}
		// raw image have been already cropped.
		else if(width == frame.cols && height == frame.rows)
		{
			offsetX = 0;
			offsetY = 0;
			need_crop = false;
		}
		else
		{
			need_crop = true;
		}
	}
	
	// roi
	cv::Rect roi(offsetX, offsetY, width, height);


	// 
	while(captureFlag_)
	{
		// image load
		std::lock_guard<std::mutex> lock(mtx);
		std::string filename = raw_fname_first + std::to_string(image_load_count++) + raw_fname_latter;
		cv::Mat frame;
		frame = cv::imread(filename, cv::IMREAD_GRAYSCALE);

		// repeat
		if (frame.empty())
		{
			image_load_count = 0;
			filename = raw_fname_first + std::to_string(image_load_count++) + raw_fname_latter;
			frame = cv::imread(filename, cv::IMREAD_GRAYSCALE);
			std::cout << "/***************/\nEnd of raw image. repeat.\n/***************/\n";
		}

		// add
		if(need_crop)
		{
			locked_queue->enqueue(frame(roi).clone());
		}else
		{
			locked_queue->enqueue(frame.clone());
		}
	}
	return;
}
//-----------------------------------------------------------------------------
void PointGreyCameraRawImages::stopCapture() {

	captureFlag_ = false;
	for(auto &it : ths)
	{
		it.join();
	}
	
	return;
}
//-----------------------------------------------------------------------------
bool PointGreyCameraRawImages::error() {

	return false;
}
//-----------------------------------------------------------------------------
bool PointGreyCameraRawImages::set_property(const std::string &prop_name,
	std::map<std::string, std::string> &values)
{
/*	if (prop_name == "brightness")
	{
		bool use = false;
		bool mode_auto = true;
		float val = 0.0f;
		if (values.find("value") != values.end())
		{
			val = std::stof(values["value"]);
			use = true;
		}
		if (values.find("auto") != values.end())
		{
			if (values["auto"] == "false") mode_auto = false;
		}
		if (use) set_brightness(mode_auto, val);
	}
	else if (prop_name == "whitebalance") {
		bool red_use = false;
		bool blue_use = false;
		bool mode_auto = true;
		float red = 0.0f;
		float blue = 0.0f;
		if (values.find("red") != values.end())
		{
			red = std::stof(values["red"]);
			red_use = true;
		}
		if (values.find("blue") != values.end())
		{
			blue = std::stof(values["blue"]);
			blue_use = true;
		}
		if (values.find("auto") != values.end())
		{
			if (values["auto"] == "false") mode_auto = false;
		}
		if (red_use && blue_use) set_white_balance(mode_auto, red, blue);
	}
	else */if (prop_name == "framerate")
	{
		bool use = false;
		bool mode_auto = true;
		float val = 0.0f;
		if (values.find("value") != values.end())
		{
			val = std::stof(values["value"]);
			use = true;
		}
		if (values.find("auto") != values.end())
		{
			if (values["auto"] == "false") mode_auto = false;
		}
		if (use) set_frame_rate(double(val));
	}
	else
	{
		return false;
	}

	return true;
}
//-----------------------------------------------------------------------------
bool PointGreyCameraRawImages::get_property(const std::string &prop_name,
	std::string &values)
{
	values = "[w] Grasshopper::get_property: Unknown param " + prop_name;
	return true;
}

//-----------------------------------------------------------------------------
void PointGreyCameraRawImages::set_frame_rate(const double val)
{
	fps = val;
}
//-----------------------------------------------------------------------------
double PointGreyCameraRawImages::get_frame_rate()
{
	return fps;
}
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// 空です
bool PointGreyCameraRawImages::set(int propId, double value){
	//switch (propId)
	//{
	//default:
	//	break;
	//}
	return false;
}
//-----------------------------------------------------------------------------
// 空です
double PointGreyCameraRawImages::get(int propId) {
	return -1;
}
//-----------------------------------------------------------------------------
// 空です
bool PointGreyCameraRawImages::get_info(std::string &info){
	return false;
}
//-----------------------------------------------------------------------------
// 空です
void* PointGreyCameraRawImages::camera(void)
{
	return static_cast<void*>(nullptr);
}