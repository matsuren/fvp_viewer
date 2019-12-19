/**
* @file GeneralCamera.cpp
* @brief Body of the related header file functions.
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
*/

/*-------------
// 一台のGeneralCameraを用いるクラス
----------------*/

#include <iostream>
#include "GeneralCamera.hpp"
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>


// コンストラクタ


//-----------------------------------------------------------------------------
GeneralCamera::GeneralCamera(int id)
{
	camera_ = new cv::VideoCapture(id);
}
//-----------------------------------------------------------------------------
GeneralCamera::GeneralCamera(std::string filename)
{
	camera_ = new cv::VideoCapture(filename);
}

//-----------------------------------------------------------------------------
GeneralCamera::GeneralCamera()
{
	camera_ = new cv::VideoCapture();
}
//-----------------------------------------------------------------------------
GeneralCamera::~GeneralCamera(void)
{

}

//-----------------------------------------------------------------------------
int GeneralCamera::grab(void)
{
	return camera_->grab();

}

//-----------------------------------------------------------------------------
bool GeneralCamera::retrieve(cv::Mat& image)
{
	bool flag = camera_->retrieve(image);
	return flag;
}

bool GeneralCamera::set(int propId, double value){
	return camera_->set(propId, value);
}
double GeneralCamera::get(int propId){
	return camera_->get(propId);
}