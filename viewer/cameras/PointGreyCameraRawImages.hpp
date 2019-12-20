#pragma  once
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <opencv2/core.hpp>
#include "AbstractCamera.hpp"
#include "LockedQueue.hpp"

/** @brief Class to manage the PointGreyCameraRawImages camera
*/
class PointGreyCameraRawImages : public AbstractCamera
{
public:

	PointGreyCameraRawImages(const std::string, std::map<std::string, std::string> &values = std::map<std::string, std::string>());
	~PointGreyCameraRawImages(void);

	void initializeCapture(std::map<std::string, std::string> &values);
	bool SetProperty(const std::map<std::string, std::string> &values);

	/** @brief Interface to set a property with multiple values
	*/
	bool set_property(const std::string &prop_name,
		std::map<std::string, std::string> &values);

	bool get_property(const std::string &prop_name,
		std::string &values);

	
	/** @brief Grab a single image and memorize to internal buffer.
	
		Grab a single image and memorize to internal buffer.
		@return	Return 1 in case of success. 0 otherwise.
	*/
	int grab(void) override;
	int grabRaw(void) override;
	bool retrieve(cv::Mat& image) override;

	// ‹ó‚Å‚·
	bool set(int propId, double value);
	// ‹ó‚Å‚·
	double get(int propId);
	void* camera(void);


	bool error();

	/** @brief Get info about the current camera
	*/
	bool get_info(std::string &info);

private:
	//OpenCV
	cv::Mat frame_;
	bool captureFlag_ = false;
	double fps = 30;
	int image_load_count = 0;
	int image_show_count = 0;
	std::vector<std::thread> ths;
	std::mutex mtx;

	// 
	LockedQueue<cv::Mat>* locked_queue;
	const int MAX_QUEUE_SIZE = 120;

	// roi
	int offsetX = -1;
	int offsetY = -1;
	int width = -1;
	int height = -1;

	// raw data file name
	// ex. raw_data/0_*.pgm
	// raw_fname_first = "raw_data/0_"
	// raw_fname_latter = ".pgm"
	std::string raw_fname_first;
	std::string raw_fname_latter;

	/** @brief Camera specific function.
	*/
	void set_frame_rate(const double val);

	/** @brief Camera specific function.
	*/
	double get_frame_rate();

	/** @brief Camera specific function.
	*/
	void queueWorker();
	void stopCapture();
};

