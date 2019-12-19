/**
* @file PointGreyCamera.cpp
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
// 一台のPointGreyCameraを用いるクラス
----------------*/

#include <iostream>
#include <opencv2\core\core.hpp>
#include "PointGreyCamera.hpp"
#include "FlyCapture2.h"


// コンストラクタ
void PointGreyCamera::initializeMember(){
	frame_ = new cv::Mat;
	captureFlag_ = false;
	busMgr_ = new FlyCapture2::BusManager();
	guid_ = new FlyCapture2::PGRGuid();
	camera_ = new FlyCapture2::Camera();
	camInfo_ = new FlyCapture2::CameraInfo();
	rawImage_ = new FlyCapture2::Image();
	rgbImage_ = new FlyCapture2::Image();
	error_ = new FlyCapture2::Error();
}
//-----------------------------------------------------------------------------
void PointGreyCamera::initializeCapture(std::map<std::string, std::string> &values) {
	if (values.find("FlyCapture2::Mode") == values.end()) {
		values["FlyCapture2::Mode"] = "Mode_0";
	}
	// Set the property
	SetProperty(values);

	// Capture
	StartCapture();
}


// IDを用いて接続
//-----------------------------------------------------------------------------
PointGreyCamera::PointGreyCamera(int id, std::map<std::string, std::string> &values)
{
	initializeMember();
	// Get camera  
    *error_ = busMgr_->GetCameraFromIndex( id, guid_ );
  
	if (*error_ != FlyCapture2::PGRERROR_OK)
    {  
		error_->PrintErrorTrace(); 
        return;  
    }     
     
		// Connect the camera
    *error_ = camera_->Connect( guid_ );
    if ( *error_ != FlyCapture2::PGRERROR_OK )
    {
		error_->PrintErrorTrace(); 
        std::cout << "Failed to connect to camera" << std::endl;     
        return;
    }
	std::cout << "Connect to camera " << id << std::endl; 
	initializeCapture(values);
}
//-----------------------------------------------------------------------------
PointGreyCamera::PointGreyCamera(std::string serial_str, std::map<std::string, std::string> &values)
{
	unsigned int serial = std::stoi(serial_str);
	initializeMember();
	// Get camera  
	*error_ = busMgr_->GetCameraFromSerialNumber(serial, guid_);

	if (*error_ != FlyCapture2::PGRERROR_OK)
	{
		error_->PrintErrorTrace();
		return;
	}

	// Connect the camera
	*error_ = camera_->Connect(guid_);
	if (*error_ != FlyCapture2::PGRERROR_OK)
	{
		error_->PrintErrorTrace();
		std::cout << "Failed to connect to camera" << std::endl;
		return;
	}
	std::cout << "Connect to camera " << serial << std::endl;


	initializeCapture(values);

}

// IDを用いて接続
//-----------------------------------------------------------------------------
PointGreyCamera::PointGreyCamera()
{
	frame_ = new cv::Mat;
	// Connect the camera
    *error_ = camera_->Connect( 0 );
    if ( *error_ != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "Failed to connect to camera" << std::endl;     
        return;
    }
	std::map<std::string, std::string> values;
	initializeCapture(values);
}



//-----------------------------------------------------------------------------
PointGreyCamera::~PointGreyCamera(void)
{
	StopCapture();
	camera_->Disconnect();
}
// カメラの情報を表示
//-----------------------------------------------------------------------------


bool PointGreyCamera::DisplayInfo(void)
{
    // Get the camera info and print it out
    *error_ = camera_->GetCameraInfo( camInfo_ );
    if ( *error_ != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from camera" << std::endl;     
        return false;
    }
    PrintCameraInfo(camInfo_ );
	return true;
}
//-----------------------------------------------------------------------------
bool PointGreyCamera::StartCapture(void)
{
	*error_ = camera_->StartCapture();
    if ( *error_ == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;     
        return false;
    }
    else if ( *error_ != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "Failed to start image capture" << std::endl;     
        return false;
    } 
	return true;
}
//-----------------------------------------------------------------------------
bool PointGreyCamera::StopCapture(void)
{
	// Stop capturing images
	FlyCapture2::Error error = camera_->StopCapture();
	if (error != FlyCapture2::PGRERROR_OK)
	{
		PrintError(error);
		return false;
	}

	// Disconnect the camera
	error = camera_->Disconnect();
	if (error != FlyCapture2::PGRERROR_OK)
	{
		PrintError(error);
		return false;
	}

	return true;
}
//-----------------------------------------------------------------------------
bool PointGreyCamera::SetProperty(const std::map<std::string, std::string> &values)
{
	// Mode
    FlyCapture2::Mode k_fmt7Mode = FlyCapture2::MODE_0;
	if (values.find("FlyCapture2::Mode") != values.end()) {
		if (values.at(std::string("FlyCapture2::Mode")) == 
			std::string("Mode_0")) {
			k_fmt7Mode = FlyCapture2::MODE_0;
		} else if (values.at("FlyCapture2::Mode") == std::string("Mode_1")) {
			k_fmt7Mode = FlyCapture2::MODE_1;
		}
	}
	// Pixel format
	FlyCapture2::PixelFormat k_fmt7PixFmt = FlyCapture2::PIXEL_FORMAT_RAW8;

    // Query for available Format 7 modes
    FlyCapture2::Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode = k_fmt7Mode;
    *error_ = camera_->GetFormat7Info( &fmt7Info, &supported );
    if (*error_ != FlyCapture2::PGRERROR_OK)
    {
        PrintError( *error_ );
        return false;
    }

    PrintFormat7Capabilities( fmt7Info );

    if ( (k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0 )
    {
        // Pixel format not supported!
		printf("Pixel format is not supported\n");
        return false;
    }
    
    FlyCapture2::Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = k_fmt7Mode;
    fmt7ImageSettings.offsetX = 0;
    fmt7ImageSettings.offsetY = 0;
    fmt7ImageSettings.width = fmt7Info.maxWidth;
    fmt7ImageSettings.height = fmt7Info.maxHeight;
    fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

	// set ROI
	if (values.find("FlyCapture2::offsetX") != values.end()) {
		fmt7ImageSettings.offsetX = std::stoi(values.at(std::string("FlyCapture2::offsetX")));
	}
	if (values.find("FlyCapture2::offsetY") != values.end()) {
		fmt7ImageSettings.offsetY = std::stoi(values.at(std::string("FlyCapture2::offsetY")));
	}
	if (values.find("FlyCapture2::width") != values.end()) {
		// width must be 32 * n
		int n = std::stoi(values.at(std::string("FlyCapture2::width")))/32;
		fmt7ImageSettings.width = n * 32;
	}
	if (values.find("FlyCapture2::height") != values.end()) {
		// height must be 2 * n
		int n = std::stoi(values.at(std::string("FlyCapture2::height"))) / 2;
		fmt7ImageSettings.height = n * 2; 
	}
	std::cout << "image offset : (" << fmt7ImageSettings.offsetX << ", " << fmt7ImageSettings.offsetY << ")   ";
	std::cout << "image size : (" << fmt7ImageSettings.width << ", " << fmt7ImageSettings.height << ") \n";

    bool valid;
    FlyCapture2::Format7PacketInfo fmt7PacketInfo;

    // Validate the settings to make sure that they are valid
    *error_ = camera_->ValidateFormat7Settings(
        &fmt7ImageSettings,
        &valid,
        &fmt7PacketInfo );
    if (*error_ != FlyCapture2::PGRERROR_OK)
    {
        PrintError( *error_ );
        return false;
    }

    if ( !valid )
    {
        // Settings are not valid
		printf("Format7 settings are not valid\n");
        return false;
    }

    // Set the settings to the camera
   * error_ = camera_->SetFormat7Configuration(
        &fmt7ImageSettings,
        fmt7PacketInfo.recommendedBytesPerPacket );
    if (*error_ != FlyCapture2::PGRERROR_OK)
    {
        PrintError( *error_ );
        return false;
    }

	return true;
}

//-----------------------------------------------------------------------------
// output int &offsetX, int &offsetY, int &width, int &height
bool PointGreyCamera::GetCurrentROI(int &offsetX, int &offsetY, int &width, int &height)
{
	FlyCapture2::Format7ImageSettings fmt7ImageSettings;
	unsigned int pPacketSize;
	float pPercentage;
	// Get the settings to the camera
	*error_ = camera_->GetFormat7Configuration(
		&fmt7ImageSettings, &pPacketSize, &pPercentage);

	offsetX = fmt7ImageSettings.offsetX;
	offsetY = fmt7ImageSettings.offsetY; 
	width = fmt7ImageSettings.width;
	height = fmt7ImageSettings.height;

	if (*error_ != FlyCapture2::PGRERROR_OK)
	{
		PrintError(*error_);
		return false;
	}
	return true;
}

//-----------------------------------------------------------------------------
// output int &offsetX, int &offsetY, int &width, int &height
bool PointGreyCamera::GetMaxImageSize(int &width, int &height)
{
	// Query for available Format 7 modes
	FlyCapture2::Format7Info fmt7Info;
	bool supported;
	*error_ = camera_->GetFormat7Info(&fmt7Info, &supported);
	if (*error_ != FlyCapture2::PGRERROR_OK)
	{
		PrintError(*error_);
		return false;
	}

	width = fmt7Info.maxWidth;
	height = fmt7Info.maxHeight;

	return true;
}

//-----------------------------------------------------------------------------
int PointGreyCamera::grabRaw(void)
{
	// Get the image
	*error_ = camera_->RetrieveBuffer( rawImage_ );
	if ( *error_ != FlyCapture2::PGRERROR_OK )
	{
		std::cout << "capture error" << std::endl;
		return 0;
	}
       
	// convert to OpenCV Mat
	//unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() /
	//						(double)rgbImage.GetRows();       
	unsigned int rowBytes = (unsigned int)rawImage_->GetReceivedDataSize() /
							(unsigned int)rawImage_->GetRows();       

	frame_[0] = cv::Mat(rawImage_->GetRows(), rawImage_->GetCols(), CV_8UC1, 
		rawImage_->GetData(),rowBytes);
	//std::cout << &frame_[0].data << std::endl;

	return 1;
}
//-----------------------------------------------------------------------------
int PointGreyCamera::grab(void)
{
	// Get the image
	*error_ = camera_->RetrieveBuffer(rawImage_);
	if (*error_ != FlyCapture2::PGRERROR_OK)
	{
		std::cout << "capture error" << std::endl;
		return 0;
	}

	//     TimeStamp timestamp = rawImage.GetTimeStamp();
	//     printf( 
	//         "microSecond %d - Second %d - TimeStamp [%d %d]\n", 
	//timestamp.microSeconds, 
	//timestamp.seconds, 
	//         timestamp.cycleSeconds, 
	//         timestamp.cycleCount);

	// convert to rgb
	rawImage_->Convert(FlyCapture2::PIXEL_FORMAT_BGR, rgbImage_);

	// convert to OpenCV Mat
	//unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() /
	//						(double)rgbImage.GetRows();       
	unsigned int rowBytes = (unsigned int)rgbImage_->GetReceivedDataSize() /
		(unsigned int)rgbImage_->GetRows();

	frame_[0] = cv::Mat(rgbImage_->GetRows(), rgbImage_->GetCols(), CV_8UC3,
		rgbImage_->GetData(), rowBytes);
	//std::cout << &frame_[0].data << std::endl;

	return 1;
}
//-----------------------------------------------------------------------------
cv::Mat& PointGreyCamera::get_cvmat(void)
{
	return frame_[0];	
}
//-----------------------------------------------------------------------------
void* PointGreyCamera::get(void) const
{
	return &(frame_[0]); // return frame;	
}
//-----------------------------------------------------------------------------
bool PointGreyCamera::retrieve(cv::Mat& image)
{
	image = *frame_;
	return true;
}
//-----------------------------------------------------------------------------
void* PointGreyCamera::camera(void)
{
	return static_cast<void*>(&camera_);
}
//-----------------------------------------------------------------------------
void PointGreyCamera::PrintBuildInfo()
{
    FlyCapture2::FC2Version fc2Version;
    FlyCapture2::Utilities::GetLibraryVersion( &fc2Version );
    char version[128];
    sprintf( 
        version, 
        "FlyCapture2 library version: %d.%d.%d.%d\n", 
        fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build );

    printf( "%s", version );

    char timeStamp[512];
    sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );

    printf( "%s", timeStamp );
}
//-----------------------------------------------------------------------------
void PointGreyCamera::PrintCameraInfo( FlyCapture2::CameraInfo* pCamInfo )
{
	/** Bayer tile formats. */
	enum BayerTileFormat
	{
		NONE, /**< No bayer tile format. */
		RGGB, /**< Red-Green-Green-Blue. */
		GRBG, /**< Green-Red-Blue-Green. */
		GBRG, /**< Green-Blue-Red-Green. */
		BGGR, /**< Blue-Green-Green-Red. */
		BT_FORCE_32BITS = FULL_32BIT_VALUE
	};


    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n"
		,
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime
		);

	char* bayer_tile_format;
	switch (pCamInfo->bayerTileFormat)
	{
		
	case NONE: /**< No bayer tile format. */
		bayer_tile_format = "No bayer tile format";
		break;
	case RGGB: /**< Red-Green-Green-Blue. */
		bayer_tile_format = "Red-Green-Green-Blue";
		break;
	case GRBG: /**< Green-Red-Blue-Green. */
		bayer_tile_format = "Green-Red-Blue-Green";
		break;
	case GBRG: /**< Green-Blue-Red-Green. */
		bayer_tile_format = "Green-Blue-Red-Green";
		break;
	case BGGR: /**< Blue-Green-Green-Red. */
		bayer_tile_format = "Blue-Green-Green-Red";
		break;
	case BT_FORCE_32BITS:
		bayer_tile_format = "BT_FORCE_32BITS";
		break;
	default:
		break;
	}
	printf("Bayer Tile Format - %s\n\n", bayer_tile_format);

}
//-----------------------------------------------------------------------------
void PointGreyCamera::PrintFormat7Capabilities( FlyCapture2::Format7Info fmt7Info )
{
    printf(
        "Max image pixels: (%u, %u)\n"
        "Image Unit size: (%u, %u)\n"
        "Offset Unit size: (%u, %u)\n"
        "Pixel format bitfield: 0x%08x\n",
        fmt7Info.maxWidth,
        fmt7Info.maxHeight,
        fmt7Info.imageHStepSize,
        fmt7Info.imageVStepSize,
        fmt7Info.offsetHStepSize,
        fmt7Info.offsetVStepSize,
        fmt7Info.pixelFormatBitField );
}
//-----------------------------------------------------------------------------
void PointGreyCamera::PrintError( FlyCapture2::Error &error )
{
    error.PrintErrorTrace();
}
//-----------------------------------------------------------------------------
bool PointGreyCamera::error() {
	if (*error_ != FlyCapture2::PGRERROR_OK) {
		error_->PrintErrorTrace();
		return false;
	}
	return false;
}
//-----------------------------------------------------------------------------
bool PointGreyCamera::set_property(const std::string &prop_name,
	std::map<std::string, std::string> &values)
{
	if (prop_name == "brightness")
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
	else if (prop_name == "framerate")
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
		if (use) set_frame_rate(mode_auto, val);
	}
	else
	{
		return false;
	}

	return true;
}
//-----------------------------------------------------------------------------
bool PointGreyCamera::get_property(const std::string &prop_name,
	std::string &values)
{
	if (prop_name == "brightness") {
		get_brightness(values);
	}
	else if (prop_name == "whitebalance") {
		get_white_balance(values);
	}
	else {
		values = "[w] Grasshopper::get_property: Unknown param " + prop_name;
	}
	return true;
}
//-----------------------------------------------------------------------------
void PointGreyCamera::set_brightness(bool mode_auto, float val)
{
	//Declare a Property 
	FlyCapture2::Property prop;
	//Define the property to adjust.
	prop.type = FlyCapture2::BRIGHTNESS;
	//Ensure the property is set up to use absolute value control.
	prop.absControl = mode_auto;
	//Set the absolute value of brightness to 0.5%.
	prop.absValue = val;
	//Set the property.
	FlyCapture2::Error error = camera_->SetProperty(&prop);
}
//-----------------------------------------------------------------------------
void PointGreyCamera::get_brightness(std::string &val)
{
	//Declare a Property 
	FlyCapture2::Property prop;
	//Define the property to adjust.
	prop.type = FlyCapture2::BRIGHTNESS;
	//Set the property.
	FlyCapture2::Error error = camera_->GetProperty(&prop);
	val = std::to_string(prop.absValue);
}
//-----------------------------------------------------------------------------
void PointGreyCamera::set_white_balance(bool onOff, float red, float blue)
{
	//Declare a Property struct.
	FlyCapture2::Property prop;
	//Define the property to adjust.
	prop.type = FlyCapture2::WHITE_BALANCE;
	//Ensure the property is on.
	prop.onOff = onOff;// false;
	//Ensure auto-adjust mode is off.
	prop.autoManualMode = false;//mode_auto;
	//Set the white balance red channel to 500.
	prop.valueA = static_cast<unsigned int>(red);
	//Set the white balance blue channel to 850.
	prop.valueB = static_cast<unsigned int>(blue);
	//Set the property.
	FlyCapture2::Error error = camera_->SetProperty(&prop);
}
//-----------------------------------------------------------------------------
void PointGreyCamera::get_white_balance(std::string &val)
{
	//Declare a Property 
	FlyCapture2::Property prop;
	//Define the property to adjust.
	prop.type = FlyCapture2::WHITE_BALANCE;
	//Set the property.
	FlyCapture2::Error error = camera_->GetProperty(&prop);
	val = std::to_string(prop.valueA) + " " + std::to_string(prop.valueB);
}
//-----------------------------------------------------------------------------
void PointGreyCamera::set_frame_rate(bool mode_auto, float val)
{
	//Declare a Property struct.
	FlyCapture2::Property prop;
	//Define the property to adjust.
	prop.type = FlyCapture2::FRAME_RATE;
	//Ensure the property is set up to use absolute value control.
	prop.absControl = mode_auto;
	//Set the absolute value of brightness to 0.5%.
	prop.absValue = val;
	//Ensure the property is on.
	prop.onOff = true;
	//Set the property.
	*error_ = camera_->SetProperty(&prop);
	if (*error_ != FlyCapture2::PGRERROR_OK)
	{
		std::cout << "set frame rate error" << std::endl;
	}
}
//-----------------------------------------------------------------------------
void PointGreyCamera::get_frame_rate(std::string &val)
{
	//Declare a Property 
	FlyCapture2::Property prop;
	//Define the property to adjust.
	prop.type = FlyCapture2::FRAME_RATE;
	//Set the property.
	FlyCapture2::Error error = camera_->GetProperty(&prop);
	val = std::to_string(prop.valueA) + " " + std::to_string(prop.valueB);
}
//-----------------------------------------------------------------------------
bool PointGreyCamera::get_info(std::string &info)
{
	char buf[2048];
	sprintf(buf, "Serial number - %u Camera model - %s",
		camInfo_->serialNumber,
		camInfo_->modelName);
	info = buf;
	return true;
}
//-----------------------------------------------------------------------------
// 空です
bool PointGreyCamera::set(int propId, double value){
	//switch (propId)
	//{
	//default:
	//	break;
	//}
	return false;
}
//-----------------------------------------------------------------------------
// 空です
double PointGreyCamera::get(int propId){
	return -1;
}