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
#pragma once

#include "AbstractCamera.hpp"


// OpenCV
namespace cv{
	class Mat;
	class VideoCapture;
}


/** @brief Class to manage the PointGreyCamera camera
*/
class GeneralCamera : public AbstractCamera
{
public:

	GeneralCamera(void);
	GeneralCamera(int);
	GeneralCamera(std::string);
	~GeneralCamera(void);



	/** @brief Grab a single image and memorize to internal buffer.
	
		Grab a single image and memorize to internal buffer.
		@return	Return 1 in case of success. 0 otherwise.
	*/
	int grab(void);
	cv::Mat& get_cvmat(void);
	void* get(void) const;
	bool retrieve(cv::Mat& image);

	bool set(int propId, double value);
	double get(int propId);

	/** @brief Set the camera property.

	Set the camera property.
	@param[in] values The parameters to set. First: Name of the parameter.
	Second: The value to associate.
	*/
	bool SetProperty(
		const std::map<std::string, std::string> &values) {
		return true;
	}

	/** @brief Get the pointer to the current camera object.

	Get the pointer to the current camera object.
	*/
	void* camera(void) {
		return nullptr;
	}

	bool set_property(const std::string &prop_name,
		std::map<std::string, std::string> &values) {
		return true;
	}

	bool get_property(const std::string &prop_name,
		std::string &values) {
		return true;
	}

	/** @brief Get information about the current camera
	*/
	bool get_info(std::string &info) {
		return true;
	}


	bool error() { return true; }


private:


	//OpenCV
	cv::Mat *frame_;
	cv::VideoCapture *camera_;
	bool captureFlag_;





};
