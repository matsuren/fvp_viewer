#include "cookbookogl.h"
#include <GLFW/glfw3.h>
#include "main.hpp"

#include "glutils.h"
#include "fvp_system.h"
#include "GLDataManager.hpp"
#include "GLCameraManager.hpp"
#include "RecordImageManager.hpp"
#include "SettingParameters.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>

//#define WIN_WIDTH 1200
//#define WIN_HEIGHT 900
#define WIN_WIDTH 720
#define WIN_HEIGHT 540

#include <sstream>
#include <atomic>
#include <iomanip>
using std::stringstream;

fvp::System* fvp_system;
GLFWwindow* window;
string title;

// ----- global extern in main.hpp--------
// thread exit
std::atomic<bool> exit_flag(false);
bool checkExit() {
	return exit_flag;
}
void threadExit() {
	exit_flag = true;
}
// render mode 1:floor 2:dome 3:LRF
int RENDER_MODE = 3;
char** g_argv;
int g_argc;

//-----------------------------------------------------------------------------
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (fvp_system == nullptr)
		return;
	if (key == GLFW_KEY_SPACE && action == GLFW_RELEASE)
		fvp_system->animate(!(fvp_system->animating()));
	if (key >= GLFW_KEY_1 && key <= GLFW_KEY_4 && action == GLFW_PRESS)
		RENDER_MODE = key - GLFW_KEY_0;
	if (key == GLFW_KEY_RIGHT && action == GLFW_RELEASE)
		GLCameraManager::getInstance().angle += 0.01f;
	if (key == GLFW_KEY_LEFT && action == GLFW_RELEASE)
		GLCameraManager::getInstance().angle -= 0.01f;
	if (key == GLFW_KEY_R && action == GLFW_RELEASE)
		if (RecordImageManager::getInstance().isRecorded()) {
			RecordImageManager::getInstance().stopRecord();
		}
		else {
			RecordImageManager::getInstance().startRecord();
		}
	if (key == GLFW_KEY_P && action == GLFW_RELEASE) {
		char* buffer;
		int width, height;
		fvp_system->getwinsize(width, height);
		buffer = (char*)calloc(4, width * height);
		glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
		cv::Mat img(height, width, CV_8UC4, buffer);
		cv::flip(img, img, 0);
		cv::cvtColor(img, img, cv::COLOR_RGBA2BGR);
		const std::string fname = "capture.jpg";
		cv::imwrite(fname, img);
		std::cout << "Capture screen: " << fname << std::endl;
	}
}
//-----------------------------------------------------------------------------
static void scroll_callback(GLFWwindow* window, double x, double y)
{
	GLCameraManager::getInstance().scrollCallback(x, y);
}
//-----------------------------------------------------------------------------
static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	double x, y;
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
	{
		glfwGetCursorPos(window, &x, &y);
		GLCameraManager::getInstance().pressedLeftButton(x, y);
		//std::cout << "x :" << x << "\t y :" << y << std::endl;
	}
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
	{
		glfwGetCursorPos(window, &x, &y);
		GLCameraManager::getInstance().releasedLeftButton(x, y);
		//std::cout << "x :" << x << "\t y :" << y << std::endl;
	}
}
//-----------------------------------------------------------------------------
static void cursor_position_callback(GLFWwindow* window, double x, double y)
{
	GLCameraManager::getInstance().cursorPositionCallback(x, y);
}
//-----------------------------------------------------------------------------
void resizeGL(int w, int h) {
	fvp_system->resize(w, h);
}
//-----------------------------------------------------------------------------
static void resize_callback(GLFWwindow* window, int width, int height)
{
	resizeGL(width, height);
}
//-----------------------------------------------------------------------------
void initializeGL() {
	glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
#ifdef _DEBUG
	glDebugMessageCallback(GLUtils::debugCallback, NULL);
	glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, GL_TRUE);
	glDebugMessageInsert(GL_DEBUG_SOURCE_APPLICATION, GL_DEBUG_TYPE_MARKER, 0,
		GL_DEBUG_SEVERITY_NOTIFICATION, -1, "Start debugging");
#endif
	fvp_system->initScene();
}
//-----------------------------------------------------------------------------
void mainLoop() {
	const int samples = 100;
	float time[samples];
	int index = 0;

	while (!glfwWindowShouldClose(window) && !glfwGetKey(window, GLFW_KEY_ESCAPE)) {
		GLUtils::checkForOpenGLError(__FILE__, __LINE__);
		fvp_system->update(float(glfwGetTime()));
		fvp_system->render();
		glfwSwapBuffers(window);
		glfwPollEvents();

		// Update FPS
		time[index] = float(glfwGetTime());
		index = (index + 1) % samples;

		if (index == 0) {
			float sum = 0.0f;
			for (int i = 0; i < samples - 1; i++)
				sum += time[i + 1] - time[i];
			float fps = samples / sum;

			stringstream strm;
			strm << title;
			strm.precision(4);
			strm << " (fps: " << fps << ")";


			if (RecordImageManager::getInstance().isRecorded()) {
				strm << "  ****** Recording now ******   'r' : stop record";
			}
			else {
				strm << "   'r' : start record";
			}
			strm << std::fixed << std::setprecision(2)
				<< "   recorded time : " << RecordImageManager::getInstance().getElapsedSec() << "s";
			glfwSetWindowTitle(window, strm.str().c_str());
		}
	}

	threadExit();
}
//-----------------------------------------------------------------------------
/** @brief It displays if an error is raised by the glfw
*/
void error_callback(int error, const char* description)
{
	fprintf(stderr, "Error: %s\n", description);
}
//-----------------------------------------------------------------------------

int main(int argc, char* argv[])
{
	g_argc = argc;
	g_argv = argv;

	std::cout << "Free viewpoint image generation" << std::endl;

	auto cfg = std::make_shared<fvp::Config>("../../config_FVP_parameters.json");
	fvp_system = new fvp::System(cfg);
	auto manager = std::make_shared<fvp::GLDataManager>(cfg);
	fvp_system->setSensorDataManager(manager);
	// Initialize GLFW
	if (!glfwInit())
		return -1;

	// Select OpenGL 4.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, true);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, true);
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);
	// uncomment to remove the title bar
	// glfwWindowHint(GLFW_DECORATED, GL_FALSE);
	// set an error callback
	glfwSetErrorCallback(error_callback);

	// Open the window
	title = "Free viewpoint image generation";
	window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, title.c_str(), NULL, NULL);
	if (!window) {
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	// glfwSwapInterval(0)  doesn't wait for refresh
	glfwSwapInterval(1);
	glfwSetKeyCallback(window, key_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorPosCallback(window, cursor_position_callback);
	// set window size
	glfwSetWindowSizeCallback(window, resize_callback);


	// Load the OpenGL functions.
	if (!gladLoadGL()) { return -1; }
	GLUtils::dumpGLInfo();

	// Initialization
	initializeGL();
	resizeGL(WIN_WIDTH, WIN_HEIGHT);

	// Enter the main loop
	mainLoop();

	// Close window and terminate GLFW
	glfwTerminate();
	// Exit program
	return 0;
}


