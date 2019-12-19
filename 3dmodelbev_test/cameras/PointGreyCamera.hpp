/**
* @file PointGreyCamera.hpp
* @brief Class to manage the PointGreyCamera camera.
*
* @section LICENSE
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR/AUTHORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* @author uknown
* @bug No known bugs.
* @version 0.1.0.0
*
* @link : http://www.manualslib.com/manual/821605/Point-Grey-Flea3-Gige.html?page=99
*/

#ifndef TODAI_GRASSHOPPER_HPP__
#define TODAI_GRASSHOPPER_HPP__

#include "AbstractCamera.hpp"
#include <map>

//PointGrey
namespace FlyCapture2{
	class Error;
	class Camera;
	struct CameraInfo;
	class PGRGuid;
	class BusManager;
	class Image;
	struct Format7Info;
}

// OpenCV
namespace cv{
	class Mat;
}


/** @brief Class to manage the PointGreyCamera camera
*/
class PointGreyCamera : public AbstractCamera
{
public:

	PointGreyCamera();
	PointGreyCamera(int, std::map<std::string, std::string> &values = std::map<std::string, std::string>());
	PointGreyCamera(std::string, std::map<std::string, std::string> &values = std::map<std::string, std::string>());
	~PointGreyCamera(void);

	bool DisplayInfo(void);
	bool StartCapture(void);
	bool StopCapture(void);
	bool SetProperty(const std::map<std::string, std::string> &values);
	void initializeMember();
	void initializeCapture(std::map<std::string, std::string> &values);


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
	int grab(void);
	int grabRaw(void);
	cv::Mat& get_cvmat(void);
	void* get(void) const;
	bool retrieve(cv::Mat& image);

	// ‹ó‚Å‚·
	bool set(int propId, double value);
	// ‹ó‚Å‚·
	double get(int propId);

	/** @brief Get the pointer to the current camera object.
	*/
	void* camera(void);

	bool error();

	/** @brief Get info about camera roi
	*/
	bool GetCurrentROI(int &offsetX, int &offsetY, int &width, int &height);
	bool GetMaxImageSize(int &width, int &height);

private:

	//PointGrey
	FlyCapture2::Error *error_;
	FlyCapture2::Camera *camera_;
	FlyCapture2::CameraInfo *camInfo_;
    FlyCapture2::PGRGuid *guid_;  
	FlyCapture2::BusManager *busMgr_; 
	FlyCapture2::Image *rawImage_;
	FlyCapture2::Image *rgbImage_;

	//OpenCV
	cv::Mat *frame_;	
	bool captureFlag_;


	void PrintBuildInfo();

	void PrintCameraInfo( FlyCapture2::CameraInfo* pCamInfo );

	void PrintFormat7Capabilities( FlyCapture2::Format7Info fmt7Info );

	void PrintError( FlyCapture2::Error &error );

	/** @brief Camera specific function.
	*/
	void set_brightness(bool mode_auto, float val);

	/** @brief Camera specific function.
	*/
	void get_brightness(std::string &val);

	/** @brief Camera specific function.
	*/
	void set_white_balance(bool mode_auto, float red, float blue);

	/** @brief Camera specific function.
	*/
	void get_white_balance(std::string &val);

	/** @brief Camera specific function.
	*/
	void set_frame_rate(bool mode_auto, float val);

	/** @brief Camera specific function.
	*/
	void get_frame_rate(std::string &val);


	/** @brief Get info about the current camera
	*/
	bool get_info(std::string &info);


};

#endif