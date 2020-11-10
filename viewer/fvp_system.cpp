#include "fvp_system.h"
#include "GLDataManager.hpp"
#include "SettingParameters.hpp"

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

#include "GLCameraManager.hpp"
#include "GLModelManager.hpp"

namespace fvp {
	System::System(const std::shared_ptr<Config> &config) : cfg(config), m_animate(false) {
		gl_model_mgr = std::make_unique<GLModelManager>();
	}
	//-----------------------------------------------------------------------------
	void System::initScene()
	{
		const int CAMERA_NUM = 4;
		compileAndLinkShader();

		glEnable(GL_DEPTH_TEST);

		const std::string robot_model_file = cfg->robot_model_filename();
		assimp_robot = new Mesh(robot_model_file);
		assimp_robot->setProgram(&prog_robot);
		gl_model_mgr->setDrawableModel("robot", assimp_robot);

		////////////////////////////////
		// Load GL model
		////////////////////////////////
		dome = new GLModelDome(2.0f, 50);
		gl_model_mgr->setDrawableModel("dome", dome);

		float plane_size = 10.0f;
		plane_floor = new VBOPlane(plane_size, plane_size, 1, 1);
		gl_model_mgr->setDrawableModel("floor", plane_floor);

		////////////////////////////////
		// Load model matrix
		////////////////////////////////
		mat4 tmp_model_mat = mat4(1.0f);
		tmp_model_mat *= glm::translate(vec3(0.0f, 0.0f, 0.0f));
		tmp_model_mat *= glm::rotate(glm::radians(90.0f), vec3(1.0f, 0.0f, 0.0f));
		gl_model_mgr->setModelMatrix("dome", tmp_model_mat);
		gl_model_mgr->setModelMatrix("floor", tmp_model_mat);

		double lrf_x, lrf_y, lrf_rad;
		tmp_model_mat = mat4(1.0f);
		cfg->getLRFPose(lrf_x, lrf_y, lrf_rad);
		tmp_model_mat *= glm::translate(vec3(lrf_x, lrf_y, 0.0f));
		tmp_model_mat *= glm::rotate(float(lrf_rad), vec3(0.0f, 0.0f, 1.0f));
		gl_model_mgr->setModelMatrix("LRF", tmp_model_mat);

		tmp_model_mat = mat4(1.0f);
		tmp_model_mat *= glm::rotate(glm::radians(180.0f), vec3(1.0f, 0.0f, 0.0f));

		cv::Mat cv_model_mat;
		cfg->getRobotPose(cv_model_mat);
		gl_model_mgr->setModelMatrix("robot", cv_model_mat);
		prog.use();

		// Load sample images into GPU
		gl_data_mgr->initializeFisheye(prog);
		gl_data_mgr->initializeLRF();

		prog.printActiveAttribs();
		prog.printActiveUniformBlocks();
		prog.printActiveUniforms();
		std::cout << std::endl;

		// -------------------------------------------------
		// set Camera View Matrix  ---------
		{
			std::string yml_fname = cfg->cam_pose_filename();
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
				gl_data_mgr->setCameraViewMatrix(pose, i);
			}
		}

		for (int i = 0; i < CAMERA_NUM; i++) {
			cv::Mat tmp_viewmat;
			gl_data_mgr->getCameraViewMatrix(tmp_viewmat, i);
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
	}
	//-----------------------------------------------------------------------------
	void System::update(float t)
	{
		GLCameraManager::getInstance().update(t);
		gl_model_mgr->update(t);
		gl_data_mgr->update(t);
	}
	//-----------------------------------------------------------------------------
	void System::render()
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
			ModelMatrix = gl_model_mgr->getModelMatrix("floor");
			setMatrices();
			gl_model_mgr->drawModel("floor");
		}
		else if (RENDER_MODE == 2)
		{
			ModelMatrix = gl_model_mgr->getModelMatrix("dome");
			setMatrices();
			gl_model_mgr->drawModel("dome");
		}
		else if (RENDER_MODE == 3)
		{
			ModelMatrix = gl_model_mgr->getModelMatrix("LRF");
			setMatrices();
			gl_data_mgr->drawModel("LRF");
		}

		prog_robot.use();
		// draw robot
		ModelMatrix = gl_model_mgr->getModelMatrix("robot");
		setMatricesPassthrough();
		gl_model_mgr->drawModel("robot");


	}
	//-----------------------------------------------------------------------------
	void System::setMatrices()
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
	void System::setMatricesPassthrough()
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
	void System::resize(int w, int h)
	{
		glViewport(0, 0, w, h);
		win_width = w;
		win_height = h;
		ProjMatrix = glm::perspective(glm::radians(47.0f), (float)w / h, 0.3f, 1000.0f);
	}
	//-----------------------------------------------------------------------------
	void System::getwinsize(int &w, int &h)
	{
		w = win_width;
		h = win_height;
	}
	//-----------------------------------------------------------------------------
	void System::compileAndLinkShader()
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

}
