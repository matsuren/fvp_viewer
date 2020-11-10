#pragma once
#include <array>
#include <memory>
#include <opencv2/core.hpp>
#include <glm/glm.hpp>
#include "glslprogram.h"
#include "vboplane.h"
#include "Mesh.h"
// OpenGL headers
#include "cookbookogl.h"

using glm::mat4;

namespace fvp {
	class Config;
	class GLDataManager;
	class GLModelManager;

class System
{
private:
	const std::shared_ptr<Config> cfg;
	std::shared_ptr<GLDataManager> gl_data_mgr;
	std::unique_ptr<GLModelManager> gl_model_mgr;
	int win_width;
	int win_height;
	int img_width;
	int img_height;

	GLSLProgram prog;
	GLSLProgram prog_robot;

	VBOPlane *plane_floor;
    VBOPlane *plane_wall;
	VBOPlane *plane_wall2;
	Drawable *dome;
	Mesh *assimp_robot;

    mat4 ModelMatrix;
	mat4 ViewMatrix;
	std::array<mat4, 4> fisheye_views;
	std::array<cv::Mat, 4> fisheye_views_cvmat;
    mat4 ProjMatrix;


    void setMatrices();
	void setMatricesPassthrough();
    void compileAndLinkShader();

public:
    System(const std::shared_ptr<Config> &config);

	void setSensorDataManager(std::shared_ptr<GLDataManager> &manager) {
		gl_data_mgr = manager;
	}
    void initScene();
    void update( float t );
    void render();
    void resize(int, int);
	void getwinsize(int &w, int &h);


	void animate(bool value) { m_animate = value; }
	bool animating() { return m_animate; }

protected:
	bool m_animate;
};
}
