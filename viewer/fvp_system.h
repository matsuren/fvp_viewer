#pragma once
#include <array>
#include <opencv2/core.hpp>
#include <glm/glm.hpp>
#include "glslprogram.h"
#include "vboplane.h"
#include "Mesh.h"
// OpenGL headers
#include "cookbookogl.h"

using glm::mat4;

class FVPSystem
{
private:
	GLSLProgram prog;
	GLSLProgram prog_robot;

    int width, height;
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
    FVPSystem();

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
