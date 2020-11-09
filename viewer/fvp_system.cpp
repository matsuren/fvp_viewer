#include "fvp_system.h"
#include "SensorDataManager.hpp"

#include <cstdio>
#include <cstdlib>
#include <array>
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include "glutils.h"

using glm::vec3;

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "models/GLModelDome.hpp"
#include "ocam_functions.h"
#include "GLCameraManager.hpp"
#include "GLModelManager.hpp"

FVPSystem::FVPSystem() : m_animate(false) {
}
//-----------------------------------------------------------------------------
void FVPSystem::initScene()
{
	const int CAMERA_NUM = 4;
	compileAndLinkShader();

	glEnable(GL_DEPTH_TEST);

	const std::string robot_model_file = SensorDataManager::getInstance().setting_params.robot_model();
	assimp_robot = new Mesh(robot_model_file);
	assimp_robot->setProgram(&prog_robot);
	GLModelManager::getInstance().setDrawableModel("robot", assimp_robot);

	////////////////////////////////
	// Load GL model
	////////////////////////////////
	dome = new GLModelDome(2.0f, 50);
	GLModelManager::getInstance().setDrawableModel("dome", dome);

	float plane_size = 10.0f;
	plane_floor = new VBOPlane(plane_size, plane_size, 1, 1);
	GLModelManager::getInstance().setDrawableModel("floor", plane_floor);

	////////////////////////////////
	// Load model matrix
	////////////////////////////////
	mat4 tmp_model_mat = mat4(1.0f);
	tmp_model_mat *= glm::translate(vec3(0.0f, 0.0f, 0.0f));
	tmp_model_mat *= glm::rotate(glm::radians(90.0f), vec3(1.0f, 0.0f, 0.0f));
	GLModelManager::getInstance().setModelMatrix("dome", tmp_model_mat);
	GLModelManager::getInstance().setModelMatrix("floor", tmp_model_mat);

	double lrf_x, lrf_y, lrf_rad;
	tmp_model_mat = mat4(1.0f);
	SensorDataManager::getInstance().setting_params.lrf_align(lrf_x, lrf_y, lrf_rad);
	tmp_model_mat *= glm::translate(vec3(lrf_x, lrf_y, 0.0f));
	tmp_model_mat *= glm::rotate(float(lrf_rad), vec3(0.0f, 0.0f, 1.0f));
	GLModelManager::getInstance().setModelMatrix("LRF", tmp_model_mat);

	tmp_model_mat = mat4(1.0f);
	tmp_model_mat *= glm::rotate(glm::radians(180.0f), vec3(1.0f, 0.0f, 0.0f));
	
	cv::Mat cv_model_mat;
	SensorDataManager::getInstance().setting_params.robot_align(cv_model_mat);
	GLModelManager::getInstance().setModelMatrix("robot", cv_model_mat);

	prog.use();
	// Load texture file to texture array
	glActiveTexture(GL_TEXTURE0);
	GLuint texID;
	glGenTextures(1, &texID);
	glBindTexture(GL_TEXTURE_2D_ARRAY, texID);
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

	// Load image
	for (int i = 0; i < CAMERA_NUM; i++)
	{
		cv::Mat mat_data;
		if (SensorDataManager::getInstance().getCameraImage(mat_data, i) != 0)
			exit(EXIT_FAILURE);
		width = mat_data.cols;
		height = mat_data.rows;

		// allocate memory
		if (i == 0) {
			glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGB, width, height, CAMERA_NUM, 0, GL_BGR, GL_UNSIGNED_BYTE, nullptr);
			// glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_RGB8, width, height, CAMERA_NUM);
		}

		//create a pixel buffer object. you need to delete them when program exits.
		GLuint pixel_buffer;
		glGenBuffers(1, &pixel_buffer);
		SensorDataManager::getInstance().setPixelbuffer(pixel_buffer, i);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pixel_buffer);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, 3 * width*height, mat_data.data, GL_DYNAMIC_DRAW);
		// send data
		glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, i, width, height, 1, GL_BGR, GL_UNSIGNED_BYTE, nullptr);
	}

	// for OCamCalib
	for (int i = 0; i < CAMERA_NUM; i++)
	{
		/* --------------------------------------------------------------------*/
		/* Read the parameters of the omnidirectional camera from the TXT file */
		/* --------------------------------------------------------------------*/
		struct ocam_model o; // our ocam_models for the fisheye and catadioptric cameras
							 //get_ocam_model(&o, DATA_PATH"/data/calib_results_fisheye.txt");
		std::string ocam_fname = SensorDataManager::getInstance().setting_params.calibfile(i);
		if (get_ocam_model(&o, ocam_fname.c_str()) == -1)
		{
			exit(EXIT_FAILURE);
		}

		/* --------------------------------------------------------------------*/
		/* Print ocam_model parameters                                         */
		/* --------------------------------------------------------------------*/
		std::string ocamparam_name = "OCamParams[" + std::to_string(i) + "]";
		std::cout << "OcamParam name : " << ocamparam_name << std::endl;
		const int INVPOL_MAX = 14;
		const int POL_MAX = 6;
		while (o.length_invpol < INVPOL_MAX) {
			o.invpol[o.length_invpol] = 0;
			o.length_invpol++;
		}
		while (o.length_pol < POL_MAX) {
			o.pol[o.length_pol] = 0;
			o.length_pol++;
		}

		printf("pol =\n");    for (int i = 0; i < o.length_pol; i++) { printf("\t%e\n", o.pol[i]); };    printf("\n");
		printf("invpol =\n"); for (int i = 0; i < o.length_invpol; i++) { printf("\t%e\n", o.invpol[i]); }; printf("\n");
		printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n", o.xc, o.yc, o.width, o.height);
		prog.setUniform(ocamparam_name + ".xc", o.xc);
		prog.setUniform(ocamparam_name + ".yc", o.yc);
		prog.setUniform(ocamparam_name + ".c", o.c);
		prog.setUniform(ocamparam_name + ".d", o.d);
		prog.setUniform(ocamparam_name + ".e", o.e);
		prog.setUniform(ocamparam_name + ".width", float(o.width));
		prog.setUniform(ocamparam_name + ".height", float(o.height));
		// set array 
		prog.setUniform(ocamparam_name + ".invpol", o.invpol, INVPOL_MAX);
		prog.setUniform(ocamparam_name + ".pol", o.pol, POL_MAX);

		// set fov
		//prog.setUniform(ocamparam_name + ".fov", M_PI);
	}

	prog.setUniform("CAMERA_NUM", CAMERA_NUM);

	prog.printActiveAttribs();
	prog.printActiveUniformBlocks();
	prog.printActiveUniforms();
	std::cout << std::endl;
	
	// -------------------------------------------------
	// set Camera View Matrix  ---------
	{
		std::string yml_fname = SensorDataManager::getInstance().setting_params.cam_poses();
		cv::FileStorage fs(yml_fname, cv::FileStorage::READ);
		if (!fs.isOpened()) {
			std::string msg = "No camera pose yaml file:" + yml_fname;
			throw std::exception(msg.c_str());
		}
		for (size_t i = 0; i < CAMERA_NUM; i++)
		{
			std::string key = "img" + std::to_string(i) + "origin";
			cv::Mat pose;
			fs[key] >> pose;
			SensorDataManager::getInstance().setCameraViewMatrix(pose, i);
		}
	}

	for (int i = 0; i < CAMERA_NUM; i++) {
		cv::Mat tmp_viewmat;
		SensorDataManager::getInstance().getCameraViewMatrix(tmp_viewmat, i);
		fisheye_views[i] = glm::make_mat4(tmp_viewmat.ptr<float>(0, 0));
	}

	// print camera poses
	for (int i = 0; i < CAMERA_NUM; i++) {
		std::cout << "fisheye_views[" << std::to_string(i) << "]" << std::endl;
		for (int j = 0; j < 4; j++)
		{
			std::cout << glm::to_string(fisheye_views[i][j]) << std::endl;
		}
		std::cout << std::endl;
	}

	// environment setting
	prog_robot.use(); // Don't foget call prog.use() before call prog.set_uniform
	vec4 worldLight = vec4(5.0f, 5.0f, -4.0f, 1.0f);
	GLCameraManager::getInstance().setWorldLightPosition(worldLight);
	prog_robot.setUniform("Light.Ld", vec4(1.0f, 1.0f, 1.0f, 1.0f));
	prog_robot.setUniform("Light.Position", worldLight);
	prog_robot.setUniform("Light.La", vec4(1.0f, 1.0f, 1.0f, 1.0f));
	prog_robot.setUniform("Light.Ls", vec4(1.0f, 1.0f, 1.0f, 1.0f));

	//---------start capture camera (bool useImshow)------------
	SensorDataManager::getInstance().startCapture(false);
}
//-----------------------------------------------------------------------------
void FVPSystem::update(float t)
{
	GLCameraManager::getInstance().update(t);
	GLModelManager::getInstance().update(t);
	SensorDataManager::getInstance().update(t);
}
//-----------------------------------------------------------------------------
void FVPSystem::render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glFrontFace(GL_CW);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	// get camera view matrix
	ViewMatrix = GLCameraManager::getInstance().getViewMat();

	prog.use();

	if (RENDER_MODE == 1)
	{
		ModelMatrix = GLModelManager::getInstance().getModelMatrix("floor");
		setMatrices();
		GLModelManager::getInstance().drawModel("floor");
	}
	else if (RENDER_MODE == 2)
	{
		ModelMatrix = GLModelManager::getInstance().getModelMatrix("dome");
		setMatrices();
		GLModelManager::getInstance().drawModel("dome");
	}
	else if (RENDER_MODE == 3)
	{
		ModelMatrix = GLModelManager::getInstance().getModelMatrix("LRF");
		setMatrices();
		SensorDataManager::getInstance().drawModel("LRF");
	}

	prog_robot.use();
	// draw robot
	ModelMatrix = GLModelManager::getInstance().getModelMatrix("robot");
	setMatricesPassthrough();
	GLModelManager::getInstance().drawModel("robot");


}
//-----------------------------------------------------------------------------
void FVPSystem::setMatrices()
{
	mat4 mv = ViewMatrix * ModelMatrix;
	prog.setUniform("ModelMatrix", ModelMatrix);
	prog.setUniform("ModelViewMatrix", mv);
	for (int i = 0; i < 4; i++) {
		std::string uniform_name = "FisheyeCameraViews[" + std::to_string(i) + "]";
		prog.setUniform(uniform_name.c_str(), fisheye_views[i] * ModelMatrix);
	}
	prog.setUniform("NormalMatrix",
		mat3(vec3(mv[0]), vec3(mv[1]), vec3(mv[2])));
	prog.setUniform("MVP", ProjMatrix * mv);
}
//-----------------------------------------------------------------------------
void FVPSystem::setMatricesPassthrough()
{
	prog_robot.setUniform("PointSize", 8.0f);
	ViewMatrix = GLCameraManager::getInstance().getViewMat();
	mat4 mv = ViewMatrix * ModelMatrix;
	prog_robot.setUniform("ModelViewMatrix", mv);
	prog_robot.setUniform("NormalMatrix",
		mat3(vec3(mv[0]), vec3(mv[1]), vec3(mv[2])));
	prog_robot.setUniform("MVP", ProjMatrix * mv);

	// light
	vec4 worldLight = GLCameraManager::getInstance().getWorldLightPosition();
	prog_robot.setUniform("Light.Position", ViewMatrix * worldLight);
}
//-----------------------------------------------------------------------------
void FVPSystem::resize(int w, int h)
{
	glViewport(0, 0, w, h);
	width = w;
	height = h;
	ProjMatrix = glm::perspective(glm::radians(47.0f), (float)w / h, 0.3f, 1000.0f);
}
//-----------------------------------------------------------------------------
void FVPSystem::getwinsize(int &w, int &h)
{
	w = width;
	h = height;
}
//-----------------------------------------------------------------------------
void FVPSystem::compileAndLinkShader()
{
	try {
		prog.compileShader("../../shader/projtex.vs");
		prog.compileShader("../../shader/projtex.fs");
		prog.link();
		prog.use();

		prog_robot.compileShader("../../shader/simpleAssimpShader.vert");
		prog_robot.compileShader("../../shader/simpleAssimpShader.frag");
		prog_robot.link();
		//prog_robot.use();
	}
	catch (GLSLProgramException & e) {
		cerr << e.what() << endl;
		exit(EXIT_FAILURE);
	}
}
//-----------------------------------------------------------------------------
