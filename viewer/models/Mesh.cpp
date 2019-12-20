#include "Mesh.h"

#include <iostream>
#include <fstream>

#include "assimp\Importer.hpp"
#include "assimp\postprocess.h"
#include "assimp\DefaultLogger.hpp"

#include <opencv2\imgproc.hpp>
#include <opencv2\imgcodecs.hpp>
#include <opencv2\highgui.hpp>



/**
*	Constructor, loading the specified aiMesh
**/
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
		float *vertices = new float[mesh->mNumVertices * 3];
		for (size_t i = 0; i < mesh->mNumVertices; ++i) {
			vertices[i * 3] = mesh->mVertices[i].x;
			vertices[i * 3 + 1] = mesh->mVertices[i].y;
			vertices[i * 3 + 2] = mesh->mVertices[i].z;
			if (PRINT_FLAG)
				std::cout << "vertices : " << mesh->mVertices[i].x << ", " << mesh->mVertices[i].y << ", " << mesh->mVertices[i].z << ", " << std::endl;

		}

		glGenBuffers(1, &vbo[VERTEX_BUFFER]);
		glBindBuffer(GL_ARRAY_BUFFER, vbo[VERTEX_BUFFER]);
		glBufferData(GL_ARRAY_BUFFER, 3 * mesh->mNumVertices * sizeof(GLfloat), vertices, GL_STATIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(0);

		delete vertices;
	}


	if (mesh->HasTextureCoords(0)) {
		float *texCoords = new float[mesh->mNumVertices * 2];
		for (size_t i = 0; i < mesh->mNumVertices; ++i) {
			texCoords[i * 2] = mesh->mTextureCoords[0][i].x;
			texCoords[i * 2 + 1] = mesh->mTextureCoords[0][i].y;
			if (PRINT_FLAG)
				std::cout << "texcoord : " << texCoords[i * 2] << ", " << texCoords[i * 2 + 1] << std::endl;
		}

		glGenBuffers(1, &vbo[TEXCOORD_BUFFER]);
		glBindBuffer(GL_ARRAY_BUFFER, vbo[TEXCOORD_BUFFER]);
		glBufferData(GL_ARRAY_BUFFER, 2 * mesh->mNumVertices * sizeof(GLfloat), texCoords, GL_STATIC_DRAW);

		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(1);

		delete texCoords;
	}


	if (mesh->HasNormals()) {
		float *normals = new float[mesh->mNumVertices * 3];
		for (size_t i = 0; i < mesh->mNumVertices; ++i) {
			normals[i * 3] = mesh->mNormals[i].x;
			normals[i * 3 + 1] = mesh->mNormals[i].y;
			normals[i * 3 + 2] = mesh->mNormals[i].z;
		}

		glGenBuffers(1, &vbo[NORMAL_BUFFER]);
		glBindBuffer(GL_ARRAY_BUFFER, vbo[NORMAL_BUFFER]);
		glBufferData(GL_ARRAY_BUFFER, 3 * mesh->mNumVertices * sizeof(GLfloat), normals, GL_STATIC_DRAW);

		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(2);

		delete normals;
	}


	if (mesh->HasFaces()) {
		render_mode = GL_TRIANGLES;
		elementCount = mesh->mNumFaces * 3;
		unsigned int *indices = new unsigned int[mesh->mNumFaces * 3];
		for (size_t i = 0; i < mesh->mNumFaces; ++i) {
			indices[i * 3] = mesh->mFaces[i].mIndices[0];
			indices[i * 3 + 1] = mesh->mFaces[i].mIndices[1];
			indices[i * 3 + 2] = mesh->mFaces[i].mIndices[2];
			if (PRINT_FLAG)
				std::cout << "indices : " << indices[i * 3] << ", " << indices[i * 3 + 1] << ", " << indices[i * 3 + 2] << std::endl;
		}

		glGenBuffers(1, &vbo[INDEX_BUFFER]);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[INDEX_BUFFER]);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * mesh->mNumFaces * sizeof(GLuint), indices, GL_STATIC_DRAW);

		delete indices;
	}
	else
	{
		// render_mode
		render_mode = GL_POINTS;
		render_points_flag = true;
		// get point cloud color
		if (mesh->HasPositions()) {
			float *colors = new float[mesh->mNumVertices * 4];
			for (size_t i = 0; i < mesh->mNumVertices; ++i) {
				colors[i * 4] = mesh->mColors[0][i].r;
				colors[i * 4 + 1] = mesh->mColors[0][i].g;
				colors[i * 4 + 2] = mesh->mColors[0][i].b;
				colors[i * 4 + 3] = mesh->mColors[0][i].a;
				//colors[i * 4] = 0;
				//colors[i * 4 + 1] = 0;
				//colors[i * 4 + 2] = 0;
				//colors[i * 4 + 3] = 1;
				if (PRINT_FLAG)
					std::cout << "colors : " << colors[i * 4] << ", " << colors[i * 4 + 1] << ", " << colors[i * 4 + 2] << ", " << colors[i * 4 + 3] << ", " << std::endl;

			}

			glGenBuffers(1, &vbo[COLOR_BUFFER]);
			glBindBuffer(GL_ARRAY_BUFFER, vbo[COLOR_BUFFER]);
			glBufferData(GL_ARRAY_BUFFER, 4 * mesh->mNumVertices * sizeof(GLfloat), colors, GL_STATIC_DRAW);

			glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 0, NULL);
			glEnableVertexAttribArray(3);

			delete colors;

			// 
			elementCount = mesh->mNumVertices;
			unsigned int *indices = new unsigned int[mesh->mNumVertices];
			for (size_t i = 0; i < mesh->mNumVertices; ++i) {
				indices[i] = i;
				if (PRINT_FLAG)
					std::cout << "indices : " << indices[i] << std::endl;
			}

			glGenBuffers(1, &vbo[INDEX_BUFFER]);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[INDEX_BUFFER]);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh->mNumVertices * sizeof(GLuint), indices, GL_STATIC_DRAW);

			delete indices;
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
void Mesh::MeshEntry::initMaterial(const aiMaterial *mtl)
{
	//float c[4];

	//GLenum fill_mode;
	aiColor4D diffuse;
	aiColor4D specular;
	aiColor4D ambient;
	aiColor4D emission;
	float shininess;

	unsigned int max;	// changed: to unsigned

	int texIndex = 0;
	aiString texPath;	//contains filename of texture

	texture_loaded_flag = false;
	if (AI_SUCCESS == mtl->GetTexture(aiTextureType_DIFFUSE, texIndex, &texPath))
	{
		texture_loaded_flag = true;
		//bind texture
		std::string tex_filename = basepath + texPath.data;
		cv::Mat texture_mat = cv::imread(tex_filename);
		//cv::namedWindow("test", CV_WINDOW_NORMAL);
		//cv::imshow("test", texture_mat);
		//cv::waitKey();
		//cv::flip(texture_mat, texture_mat, 0);
		//cv::imshow("test", texture_mat);
		//cv::waitKey();
		if (texture_mat.empty())
		{
			std::cout << "error! cannot load texture image : " << tex_filename << std::endl;
			texture_loaded_flag = false;
		}
		else
		{
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
			glActiveTexture(GL_TEXTURE0);
			glGenTextures(1, &textureID);
			glBindTexture(GL_TEXTURE_2D, textureID);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture_mat.cols, texture_mat.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, texture_mat.data);
			glBindTexture(GL_TEXTURE_2D, 0);
		}

	}

	if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
	{
		for (int i = 0; i < 4; i++)
			matparam.Kd[i] = diffuse[i];
	}
	if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
	{
		for (int i = 0; i < 4; i++)
			matparam.Ks[i] = specular[i];
	}
	if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
	{
		for (int i = 0; i < 4; i++)
			matparam.Ka[i] = ambient[i];
	}
	max = 1;
	if (AI_SUCCESS == aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max))
	{
		matparam.Shininess = shininess;
	}

	//float strength;
	//int two_sided;
	//int wireframe;
	//	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
//set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
//if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
//	color4_to_float4(&emission, c);
//glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

//max = 1;
//ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
//max = 1;
//ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
//if ((ret1 == AI_SUCCESS) && (ret2 == AI_SUCCESS))
//	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
//else {
//	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
//	set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
//}

//max = 1;
//if (AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
//	fill_mode = wireframe ? GL_LINE : GL_FILL;
//else
//	fill_mode = GL_FILL;
//glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

//max = 1;
//if ((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided)
//	glEnable(GL_CULL_FACE);
//else
//	glDisable(GL_CULL_FACE);
}


/**
*	Set prog.setUniform("Material.HOGE", 0.8f, 0.8f, 0.8f);
**/
void Mesh::MeshEntry::setMaterial(GLSLProgram *prog)
{
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
void Mesh::MeshEntry::setBasePath(const std::string &path) {
	basepath = path;
}

/**
*	Renders this MeshEntry
**/
void Mesh::MeshEntry::render(GLSLProgram *prog) {
	glBindVertexArray(vao);
	setMaterial(prog);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, (texture_loaded_flag ? textureID : 0));
	//glDrawElements(GL_TRIANGLES, elementCount, GL_UNSIGNED_INT, NULL);
	// ポイントスプライトの設定
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glDrawElements(render_mode, elementCount, GL_UNSIGNED_INT, NULL);
	glBindVertexArray(0);
}

/**
*	ASSIMP Logger
**/
void Mesh::createAILogger()
{
	// Change this line to normal if you not want to analyse the import process
	Assimp::Logger::LogSeverity severity = Assimp::Logger::NORMAL;
	//Assimp::Logger::LogSeverity severity = Assimp::Logger::VERBOSE;

	// Create a logger instance for Console Output
	Assimp::DefaultLogger::create("", severity, aiDefaultLogStream_STDOUT);

	// Create a logger instance for File Output (found in project folder or near .exe)
	// Assimp::DefaultLogger::create("assimp_log.txt", severity, aiDefaultLogStream_FILE);

	// Now I am ready for logging my stuff
	// Assimp::DefaultLogger::get()->info("this is my info-call");
}
/**
*	ASSIMP Logger destroy
**/
void Mesh::destroyAILogger()
{
	// Kill it after the work is done
	Assimp::DefaultLogger::kill();
}

/**
*	Mesh constructor, loads the specified filename if supported by Assimp
**/
Mesh::Mesh(const std::string &filename, bool use_pointcloud)
{
	createAILogger();

	Assimp::Importer importer;
	//const aiScene *scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_GenSmoothNormals);
	const aiScene *scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_GenSmoothNormals
		| aiProcess_FlipUVs
		//| aiProcess_GenUVCoords
		//| aiProcess_TransformUVCoords
	);
	//const aiScene *scene = importer.ReadFile(filename, aiProcessPreset_TargetRealtime_Quality);
	//const aiScene *scene = importer.ReadFile(filename, aiProcessPreset_TargetRealtime_MaxQuality);


	if (!scene) {
		printf("Unable to load mesh: %s\n", importer.GetErrorString());
	}

	std::string basepath = getBasePath(std::string(filename));

	// ply point cloud
	std::string filename_str = std::string(filename);
	if (use_pointcloud)
	{
		if (filename_str.substr(filename_str.size() - 3) == "ply") {
			aiMesh *mesh = new aiMesh();
			plyLoader(mesh, filename_str);
			meshEntries.push_back(new Mesh::MeshEntry(mesh));
			meshEntries[0]->setBasePath(basepath);
		}
	}
	else
	{
		for (size_t i = 0; i < scene->mNumMeshes; ++i) {
			meshEntries.push_back(new Mesh::MeshEntry(scene->mMeshes[i]));
			meshEntries[i]->setBasePath(basepath);
			meshEntries[i]->initMaterial(scene->mMaterials[scene->mMeshes[i]->mMaterialIndex]);
		}
	}
}
/**
*	Load PLY PointCloud file
**/
void Mesh::plyLoader(aiMesh *mesh, const std::string& filename)
{
	std::ifstream fstream(filename, std::ios::binary);
	if (!fstream.is_open()) {
		std::cout << "error! cannot open file : " << filename << std::endl;
		return;
	}
	std::string strline;
	size_t vertex_num;

	std::vector<float> vertices;
	bool ascii_flag = true;
	bool color_alpha_flag = false;

	// load header
	while (std::getline(fstream, strline)) {
		std::vector<std::string> splitline;
		splitline = split(strline, " ");
		if (splitline[0] == "end_header") break;

		if (splitline[0] == "format")
		{
			if (splitline[1] == "ascii")
			{
				std::cout << "Load ascii ply format!" << std::endl;
				ascii_flag = true;
			}
			else if (splitline[1] == "binary_little_endian")
			{
				std::cout << "Load binary_little_endian ply format!" << std::endl;
				ascii_flag = false;
			}
			else
			{
				std::cout << "Error! please load ascii or binary_little_endian ply format!" << std::endl;
				return;
			}

		}
		if (splitline[0] == "element" && splitline[1] == "vertex")
		{
			vertex_num = std::stoi(splitline[2]);
			std::cout << "vertex number : " << vertex_num << std::endl;
		}
		if (splitline[0] == "property" && splitline[1] == "uchar" && splitline[2] == "alpha")
		{
			color_alpha_flag = true;
		}

	}

	// load vertex and color
	mesh->mNumVertices = vertex_num;
	mesh->mVertices = new aiVector3D[vertex_num];
	mesh->mColors[0] = new aiColor4D[vertex_num];
	size_t v_count = 0;
	if (ascii_flag)
	{
		// ascii
		while (std::getline(fstream, strline)) {
			std::vector<std::string> splitline;
			splitline = split(strline, " ");
			float x = std::stof(splitline[0]);
			float y = std::stof(splitline[1]);
			float z = std::stof(splitline[2]);
			float r = std::stof(splitline[3]) / 255.f;
			float g = std::stof(splitline[4]) / 255.f;
			float b = std::stof(splitline[5]) / 255.f;
			float a;
			if (color_alpha_flag)
				a = std::stof(splitline[6]) / 255.f;
			else
				a = 1.0f;
			mesh->mVertices[v_count] = aiVector3t<float>(x, y, z);
			mesh->mColors[0][v_count] = aiColor4t<float>(r, g, b, a);
			v_count++;
			if (v_count == vertex_num) break;
		}
	}
	else
	{
		// binary_little_endian
		for (v_count = 0; v_count < vertex_num; v_count++) {
			float xyz[3];
			fstream.read((char*)xyz, sizeof(float) * 3);

			if (color_alpha_flag) {
				unsigned char rgba[4];
				fstream.read((char*)rgba, sizeof(unsigned char) * 4);

				float r = rgba[0] / 255.f;
				float g = rgba[1] / 255.f;
				float b = rgba[2] / 255.f;
				float a = rgba[3] / 255.f;
				mesh->mVertices[v_count] = aiVector3t<float>(xyz[0], xyz[1], xyz[2]);
				mesh->mColors[0][v_count] = aiColor4t<float>(r, g, b, a);
			}
			else {
				unsigned char rgb[3];
				fstream.read((char*)rgb, sizeof(unsigned char) * 3);

				float r = rgb[0] / 255.f;
				float g = rgb[1] / 255.f;
				float b = rgb[2] / 255.f;
				float a = 1.0f;
				mesh->mVertices[v_count] = aiVector3t<float>(xyz[0], xyz[1], xyz[2]);
				mesh->mColors[0][v_count] = aiColor4t<float>(r, g, b, a);
			}
		}
	}


	return;
}
/**
*	Spilit string
**/
std::vector<std::string> Mesh::split(const std::string& s, const std::string delim)
{
	std::vector<std::string> result;
	result.clear();

	using string = std::string;
	string::size_type pos = 0;

	while (pos != string::npos)
	{
		string::size_type p = s.find(delim, pos);

		if (p == string::npos)
		{
			result.push_back(s.substr(pos));
			break;
		}
		else {
			result.push_back(s.substr(pos, p - pos));
		}

		pos = p + delim.size();
	}

	// compress
	for (size_t i = 0; i < result.size(); i++) {
		if (result[i] == "" || result[i] == delim) {
			result.erase(result.begin() + i);
			i--;
		}
	}

	return result;
}
/**
*	Clears all loaded MeshEntries
**/
Mesh::~Mesh(void)
{
	for (size_t i = 0; i < meshEntries.size(); ++i) {
		delete meshEntries.at(i);
	}
	meshEntries.clear();
	destroyAILogger();
}

/**
*	Renders all loaded MeshEntries
**/
void Mesh::render() const {
	if (tmp_prog == nullptr) {
		std::cout
			<< "/**--------------------\n"
			<< "error! please set Program before rendering!\n"
			<< "--------------------**/\n";
	}
	for (size_t i = 0; i < meshEntries.size(); ++i) {
		meshEntries.at(i)->render(tmp_prog);
	}
}
/**
*	Set program to use setUniform
**/
void Mesh::setProgram(GLSLProgram *prog) {
	tmp_prog = prog;
}
/**
*	get basepath
**/
std::string Mesh::getBasePath(const std::string& path)
{
	size_t pos = path.find_last_of("\\/");
	return (std::string::npos == pos) ? "" : path.substr(0, pos + 1);
}