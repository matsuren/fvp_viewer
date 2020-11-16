#include "models/mesh.hpp"

#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "assimp/DefaultLogger.hpp"
#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"

namespace model {
Mesh::MeshEntry::MeshEntry(aiMesh *mesh) {
  const bool PRINT_FLAG = false;
  vbo[VERTEX_BUFFER] = NULL;
  vbo[TEXCOORD_BUFFER] = NULL;
  vbo[NORMAL_BUFFER] = NULL;
  vbo[INDEX_BUFFER] = NULL;
  vbo[COLOR_BUFFER] = NULL;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  if (mesh->HasPositions()) {
    std::vector<float> vertices(mesh->mNumVertices * 3, 0);
    for (size_t i = 0; i < mesh->mNumVertices; ++i) {
      vertices[i * 3] = mesh->mVertices[i].x;
      vertices[i * 3 + 1] = mesh->mVertices[i].y;
      vertices[i * 3 + 2] = mesh->mVertices[i].z;
      if (PRINT_FLAG)
        std::cout << "vertices : " << mesh->mVertices[i].x << ", "
                  << mesh->mVertices[i].y << ", " << mesh->mVertices[i].z
                  << ", " << std::endl;
    }

    glGenBuffers(1, &vbo[VERTEX_BUFFER]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[VERTEX_BUFFER]);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat),
                 vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(0);
  }

  if (mesh->HasTextureCoords(0)) {
    std::vector<float> texCoords(mesh->mNumVertices * 2, 0);
    for (size_t i = 0; i < mesh->mNumVertices; ++i) {
      texCoords[i * 2] = mesh->mTextureCoords[0][i].x;
      texCoords[i * 2 + 1] = mesh->mTextureCoords[0][i].y;
      if (PRINT_FLAG)
        std::cout << "texcoord : " << texCoords[i * 2] << ", "
                  << texCoords[i * 2 + 1] << std::endl;
    }

    glGenBuffers(1, &vbo[TEXCOORD_BUFFER]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[TEXCOORD_BUFFER]);
    glBufferData(GL_ARRAY_BUFFER, texCoords.size() * sizeof(GLfloat),
                 texCoords.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(1);
  }

  if (mesh->HasNormals()) {
    std::vector<float> normals(mesh->mNumVertices * 3, 0);
    for (size_t i = 0; i < mesh->mNumVertices; ++i) {
      normals[i * 3] = mesh->mNormals[i].x;
      normals[i * 3 + 1] = mesh->mNormals[i].y;
      normals[i * 3 + 2] = mesh->mNormals[i].z;
    }

    glGenBuffers(1, &vbo[NORMAL_BUFFER]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[NORMAL_BUFFER]);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(GLfloat),
                 normals.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(2);
  }

  if (mesh->HasFaces()) {
    render_mode = GL_TRIANGLES;
    elementCount = mesh->mNumFaces * 3;
    std::vector<unsigned int> indices(mesh->mNumFaces * 3, 0);
    for (size_t i = 0; i < mesh->mNumFaces; ++i) {
      indices[i * 3] = mesh->mFaces[i].mIndices[0];
      indices[i * 3 + 1] = mesh->mFaces[i].mIndices[1];
      indices[i * 3 + 2] = mesh->mFaces[i].mIndices[2];
      if (PRINT_FLAG)
        std::cout << "indices : " << indices[i * 3] << ", "
                  << indices[i * 3 + 1] << ", " << indices[i * 3 + 2]
                  << std::endl;
    }

    glGenBuffers(1, &vbo[INDEX_BUFFER]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[INDEX_BUFFER]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
                 indices.data(), GL_STATIC_DRAW);
  } else {
    // render_mode
    render_mode = GL_POINTS;
    render_points_flag = true;
    // get point cloud color
    if (mesh->HasPositions()) {
      std::vector<float> colors(mesh->mNumVertices * 4, 0);
      for (size_t i = 0; i < mesh->mNumVertices; ++i) {
        colors[i * 4] = mesh->mColors[0][i].r;
        colors[i * 4 + 1] = mesh->mColors[0][i].g;
        colors[i * 4 + 2] = mesh->mColors[0][i].b;
        colors[i * 4 + 3] = mesh->mColors[0][i].a;
        if (PRINT_FLAG)
          std::cout << "colors : " << colors[i * 4] << ", " << colors[i * 4 + 1]
                    << ", " << colors[i * 4 + 2] << ", " << colors[i * 4 + 3]
                    << ", " << std::endl;
      }

      glGenBuffers(1, &vbo[COLOR_BUFFER]);
      glBindBuffer(GL_ARRAY_BUFFER, vbo[COLOR_BUFFER]);
      glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat),
                   colors.data(), GL_STATIC_DRAW);

      glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 0, NULL);
      glEnableVertexAttribArray(3);

      //
      elementCount = mesh->mNumVertices;
      std::vector<unsigned int> indices(mesh->mNumVertices, 0);
      for (size_t i = 0; i < mesh->mNumVertices; ++i) {
        indices[i] = i;
        if (PRINT_FLAG) std::cout << "indices : " << indices[i] << std::endl;
      }

      glGenBuffers(1, &vbo[INDEX_BUFFER]);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[INDEX_BUFFER]);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
                   indices.data(), GL_STATIC_DRAW);
    }
  }

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}

/**
 *	Deletes the allocated OpenGL buffers
 **/
Mesh::MeshEntry::~MeshEntry() {
  if (vbo[VERTEX_BUFFER]) {
    glDeleteBuffers(1, &vbo[VERTEX_BUFFER]);
  }
  if (vbo[TEXCOORD_BUFFER]) {
    glDeleteBuffers(1, &vbo[TEXCOORD_BUFFER]);
  }
  if (vbo[NORMAL_BUFFER]) {
    glDeleteBuffers(1, &vbo[NORMAL_BUFFER]);
  }
  if (vbo[INDEX_BUFFER]) {
    glDeleteBuffers(1, &vbo[INDEX_BUFFER]);
  }
  glDeleteVertexArrays(1, &vao);
}

/**
 *	Renders this MeshEntry
 **/
void Mesh::MeshEntry::initMaterial(const aiMaterial *mtl) {
  // GLenum fill_mode;
  aiColor4D diffuse;
  aiColor4D specular;
  aiColor4D ambient;
  aiColor4D emission;
  float shininess;

  unsigned int max;  // changed: to unsigned

  int texIndex = 0;
  aiString texPath;  // contains filename of texture

  texture_loaded_flag = false;
  if (AI_SUCCESS ==
      mtl->GetTexture(aiTextureType_DIFFUSE, texIndex, &texPath)) {
    texture_loaded_flag = true;
    // bind texture
    std::string tex_filename = basepath + texPath.data;
    cv::Mat texture_mat = cv::imread(tex_filename);
    // cv::namedWindow("test", CV_WINDOW_NORMAL);
    // cv::imshow("test", texture_mat);
    // cv::waitKey();
    // cv::flip(texture_mat, texture_mat, 0);
    // cv::imshow("test", texture_mat);
    // cv::waitKey();
    if (texture_mat.empty()) {
      std::cout << "error! cannot load texture image : " << tex_filename
                << std::endl;
      texture_loaded_flag = false;
    } else {
      glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
      glActiveTexture(GL_TEXTURE0);
      glGenTextures(1, &textureID);
      glBindTexture(GL_TEXTURE_2D, textureID);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture_mat.cols, texture_mat.rows,
                   0, GL_BGR, GL_UNSIGNED_BYTE, texture_mat.data);
      glBindTexture(GL_TEXTURE_2D, 0);
    }
  }

  if (AI_SUCCESS ==
      aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse)) {
    for (int i = 0; i < 4; i++) matparam.Kd[i] = diffuse[i];
  }
  if (AI_SUCCESS ==
      aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular)) {
    for (int i = 0; i < 4; i++) matparam.Ks[i] = specular[i];
  }
  if (AI_SUCCESS ==
      aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient)) {
    for (int i = 0; i < 4; i++) matparam.Ka[i] = ambient[i];
  }
  max = 1;
  if (AI_SUCCESS ==
      aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max)) {
    matparam.Shininess = shininess;
  }
}

/**
 *	Set prog.setUniform("Material.HOGE", 0.8f, 0.8f, 0.8f);
 **/
void Mesh::MeshEntry::setMaterial(GLSLProgram *prog) const {
  prog->setUniform("Material.Kd", matparam.Kd);
  prog->setUniform("Material.Ka", matparam.Ka);
  prog->setUniform("Material.Ks", matparam.Ks);
  prog->setUniform("Material.Shininess", matparam.Shininess);
  prog->setUniform("TexFlag", texture_loaded_flag);
  prog->setUniform("RenderPointsFlag", render_points_flag);
}
/**
 *	Set basepath (Model folder)
 **/
void Mesh::MeshEntry::setBasePath(const std::string &path) { basepath = path; }

/**
 *	Renders this MeshEntry
 **/
void Mesh::MeshEntry::render(GLSLProgram *prog) const {
  glBindVertexArray(vao);
  setMaterial(prog);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, (texture_loaded_flag ? textureID : 0));
  // glDrawElements(GL_TRIANGLES, elementCount, GL_UNSIGNED_INT, NULL);
  //
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glDrawElements(render_mode, elementCount, GL_UNSIGNED_INT, NULL);
  glBindVertexArray(0);
}

/**
 *	ASSIMP Logger
 **/
void Mesh::createAILogger() {
  // Change this line to normal if you not want to analyse the import process
  Assimp::Logger::LogSeverity severity = Assimp::Logger::NORMAL;
  // Assimp::Logger::LogSeverity severity = Assimp::Logger::VERBOSE;

  // Create a logger instance for Console Output
  Assimp::DefaultLogger::create("", severity, aiDefaultLogStream_STDOUT);

  // Create a logger instance for File Output (found in project folder or near
  // .exe) Assimp::DefaultLogger::create("assimp_log.txt", severity,
  // aiDefaultLogStream_FILE);

  // Now I am ready for logging my stuff
  // Assimp::DefaultLogger::get()->info("this is my info-call");
}
/**
 *	ASSIMP Logger destroy
 **/
void Mesh::destroyAILogger() {
  // Kill it after the work is done
  Assimp::DefaultLogger::kill();
}

/**
 *	Mesh constructor, loads the specified filename if supported by Assimp
 **/
Mesh::Mesh(const std::string &filename, GLSLProgram *prog) : _prog(prog) {
  createAILogger();

  Assimp::Importer importer;
  const aiScene *scene = importer.ReadFile(
      filename,
      aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs
      //| aiProcess_GenUVCoords
      //| aiProcess_TransformUVCoords
  );
  // const aiScene *scene = importer.ReadFile(filename,
  // aiProcessPreset_TargetRealtime_Quality); const aiScene *scene =
  // importer.ReadFile(filename, aiProcessPreset_TargetRealtime_MaxQuality);

  if (!scene) {
    printf("Unable to load mesh: %s\n", importer.GetErrorString());
    return;
  }

  std::string basepath = getBasePath(std::string(filename));
  meshEntries.reserve(scene->mNumMeshes);
  for (size_t i = 0; i < scene->mNumMeshes; ++i) {
    meshEntries.emplace_back(scene->mMeshes[i]);
    meshEntries[i].setBasePath(basepath);
    meshEntries[i].initMaterial(
        scene->mMaterials[scene->mMeshes[i]->mMaterialIndex]);
  }
}

/**
 *	Clears all loaded MeshEntries
 **/
Mesh::~Mesh(void) {
  meshEntries.clear();
  destroyAILogger();
}

/**
 *	Renders all loaded MeshEntries
 **/
void Mesh::render() const {
  if (_prog == nullptr) {
    std::cout << "/**--------------------\n"
              << "error! please set Program before rendering!\n"
              << "--------------------**/\n";
  }
  for (size_t i = 0; i < meshEntries.size(); ++i) {
    meshEntries[i].render(_prog);
  }
}
/**
 *	Set program to use setUniform
 **/
void Mesh::setProgram(GLSLProgram *prog) { _prog = prog; }
/**
 *	get basepath
 **/
std::string Mesh::getBasePath(const std::string &path) {
  size_t pos = path.find_last_of("\\/");
  return (std::string::npos == pos) ? "" : path.substr(0, pos + 1);
}
}  // namespace model
