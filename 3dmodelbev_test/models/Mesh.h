#pragma once
#include "cookbookogl.h"
#include <GLFW/glfw3.h>

#include "glm/glm.hpp"
#include "drawable.h"

#include "glslprogram.h"
#include "assimp\scene.h"
#include "assimp\mesh.h"

#include <vector>

class Mesh : public Drawable
{
public :
	struct MeshEntry {
		static enum BUFFERS {
			VERTEX_BUFFER, TEXCOORD_BUFFER, NORMAL_BUFFER, INDEX_BUFFER, COLOR_BUFFER
		};
		GLuint vao;
		GLuint vbo[5];

		GLenum render_mode = GL_TRIANGLES;
		
		// material
		struct MaterialParam
		{
			glm::vec4 Ka = vec4(0.5f, 0.9f, 0.3f, 1.0f);
			glm::vec4 Kd = vec4(0.5f, 0.9f, 0.3f, 1.0f);
			glm::vec4 Ks = vec4(0.8f, 0.8f, 0.8f, 1.0f);
			float Shininess = 60.0f;
		};
		MaterialParam matparam;
		bool texture_loaded_flag = false;
		bool render_points_flag = false;
		std::map<std::string, GLuint*> textureIdMap;
		std::string basepath; // model folder
		GLuint textureID; // glBindTexture(GL_TEXTURE_2D, textureID);

		unsigned int elementCount;

		MeshEntry(aiMesh *mesh);
		~MeshEntry();
		void initMaterial(const aiMaterial *mtl_);
		void setMaterial(GLSLProgram *prog);
		void setBasePath(const std::string &path);

		void load(aiMesh *mesh);
		void render(GLSLProgram *prog);
	};

	std::vector<MeshEntry*> meshEntries;

	void createAILogger();
	void destroyAILogger();

public:
	Mesh(const std::string &filename, bool use_pointcloud = false);
	~Mesh(void);
	GLSLProgram *tmp_prog = nullptr; // setUniform‚Ì‚Ý‚ÉŽg—p

	void render() const;
	void setProgram(GLSLProgram *prog);
	float scale = 1.0f;

private:
	std::string getBasePath(const std::string& path);
	void plyLoader(aiMesh *mesh, const std::string& filename);
	std::vector<std::string> split(const std::string& s, const std::string delim);
};

