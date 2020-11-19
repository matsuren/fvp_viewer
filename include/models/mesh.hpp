#pragma once
// clang-format off
#include "glslcookbook/cookbookogl.h"
#include <GLFW/glfw3.h>
// clang-format on
#include <memory>
#include <vector>

#include "assimp/mesh.h"
#include "assimp/scene.h"

#include "models/drawable.hpp"
#include "glm/glm.hpp"
#include "glslcookbook/glslprogram.h"

namespace model {
class Mesh : public Drawable {
 public:
  struct MeshEntry {
    enum BUFFERS {
      VERTEX_BUFFER,
      TEXCOORD_BUFFER,
      NORMAL_BUFFER,
      INDEX_BUFFER,
      COLOR_BUFFER
    };
    GLuint vao;
    GLuint vbo[5];

    GLenum render_mode = GL_TRIANGLES;

    // material
    struct MaterialParam {
      glm::vec4 Ka = glm::vec4(0.5f, 0.9f, 0.3f, 1.0f);
      glm::vec4 Kd = glm::vec4(0.5f, 0.9f, 0.3f, 1.0f);
      glm::vec4 Ks = glm::vec4(0.8f, 0.8f, 0.8f, 1.0f);
      float Shininess = 60.0f;
    };
    MaterialParam matparam;
    bool texture_loaded_flag = false;
    bool render_points_flag = false;
    std::map<std::string, GLuint *> textureIdMap;
    std::string basepath;  // model folder
    GLuint textureID = 0;  // glBindTexture(GL_TEXTURE_2D, textureID);

    unsigned int elementCount = 0;

    MeshEntry(aiMesh *mesh);
    ~MeshEntry();
    void initMaterial(const aiMaterial *mtl_);
    void setMaterial(const std::shared_ptr<GLSLProgram> &prog) const;
    void setBasePath(const std::string &path);
    void render(const std::shared_ptr<GLSLProgram> &prog) const;
  };

  std::vector<MeshEntry> meshEntries;

  void createAILogger();
  void destroyAILogger();

 public:
  Mesh(const std::string &filename, std::shared_ptr<GLSLProgram> &prog);
  ~Mesh(void);
  const std::shared_ptr<GLSLProgram> _prog;  // Default program

  void render() const;
  float scale = 1.0f;

 private:
  std::string getBasePath(const std::string &path);
};

}  // namespace model
