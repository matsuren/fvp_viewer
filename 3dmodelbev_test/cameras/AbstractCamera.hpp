/**
* @file AbstractCamera.hpp
* @brief Class to manage a generic camera.
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
* @author Alessandro Moro, Ren Komatsu
* @bug No known bugs.
* @version 0.1.0.0
*
*/

#pragma once

#include <map>

// OpenCV
namespace cv{
	class Mat;
}

/** @brief Class to manage a generic abstract camera.
*/
class AbstractCamera
{
public:

	AbstractCamera() {}

	/** @brief Grab a single image and memorize to internal buffer.

	Grab a single image and memorize to internal buffer.
	@return	Return 1 in case of success. 0 otherwise.
	*/
	virtual int grab(void) = 0;
	virtual int grabRaw(void) = 0;
	virtual bool retrieve(cv::Mat& image) = 0;

	// 指定のプロパティ propId に指定の値 value をセットします．
	virtual bool set(int propId, double value) = 0;
	// 指定のプロパティの値を取得します．
	virtual double get(int propId) = 0;

	/** @brief Set the camera property.

		Set the camera property.
		@param[in] values The parameters to set. First: Name of the parameter.
		Second: The value to associate.
	*/
	virtual bool SetProperty(
		const std::map<std::string, std::string> &values) = 0;

	/** @brief Get the pointer to the current camera object.

		Get the pointer to the current camera object.
	*/
	virtual void* camera(void) = 0;

	virtual bool set_property(const std::string &prop_name,
		std::map<std::string, std::string> &values) = 0;

	virtual bool get_property(const std::string &prop_name,
		std::string &values) = 0;

	/** @brief Get information about the current camera
	*/
	virtual bool get_info(std::string &info) = 0;


	virtual bool error() = 0;

};
