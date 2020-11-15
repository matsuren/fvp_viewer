#include "models/dome.hpp"
#include <string>
#include <vector>

#include "glslcookbook/cookbookogl.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace model {
	Dome::Dome(float radius, int divide_num)
	{
		std::vector<float> vertices;
		std::vector<GLuint> elements;
		///////////////////////
		// draw floor
		//////////////////////
		// origin
		vertices.push_back(0.0);
		vertices.push_back(0.0);
		vertices.push_back(0.0);

		int theta_divide_num = divide_num * 2;
		for (int t = 0; t < theta_divide_num; t++)
		{
			// vertex
			float theta = float(2 * M_PI / theta_divide_num *t);
			vertices.push_back(radius * sin(theta));
			vertices.push_back(0.0);
			vertices.push_back(radius * cos(theta));

			// element
			elements.push_back(0);
			elements.push_back(t + 1);
			elements.push_back((t + 1 != theta_divide_num) ? t + 2 : 1);
		}
		///////////////////////
		// draw dome
		//////////////////////
		for (int p = 1; p < divide_num + 1; p++)
		{
			for (int t = 0; t < theta_divide_num; t++)
			{
				// vertex
				float theta = float(2 * M_PI / theta_divide_num * t);
				float phi = float(M_PI / divide_num * p);
				vertices.push_back(radius * cos(phi)* sin(theta));
				vertices.push_back(radius * sin(phi));
				vertices.push_back(radius * cos(phi)* cos(theta));

				// element
				int index = int(vertices.size() / 3 - 1);
				elements.push_back(index);
				if (t + 1 != theta_divide_num)
					elements.push_back(index - theta_divide_num + 1);
				else
					elements.push_back(index - 2 * theta_divide_num + 1);
				elements.push_back(index - theta_divide_num);
				//
				elements.push_back(index);
				if (t + 1 != theta_divide_num)
				{
					elements.push_back(index + 1);
					elements.push_back(index - theta_divide_num + 1);
				}
				else
				{
					elements.push_back(index - theta_divide_num + 1);
					elements.push_back(index - 2 * theta_divide_num + 1);
				}
			}

		}

		//// Top
		//vertices.push_back(0.0);
		//vertices.push_back( radius);
		//vertices.push_back(0.0);
		//// element
		//int top_index = int(vertices.size() / 3 - 1);
		//for (int t = 0; t < theta_divide_num; t++)
		//{
		//	elements.push_back(top_index);
		//	if (t + 1 != theta_divide_num)
		//		elements.push_back(top_index + t - theta_divide_num + 1);
		//	else
		//		elements.push_back(top_index + t - theta_divide_num);
		//	elements.push_back(top_index + t - 2 * theta_divide_num + 1);
		//}

		vertices_num = int(vertices.size());

		unsigned int handle[2];
		glGenBuffers(2, handle);

		glBindBuffer(GL_ARRAY_BUFFER, handle[0]);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle[1]);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, elements.size() * sizeof(GLuint), &elements[0], GL_STATIC_DRAW);

		glGenVertexArrays(1, &vaoHandle);
		glBindVertexArray(vaoHandle);

		glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, ((GLubyte *)NULL + (0)));
		glEnableVertexAttribArray(0);  // Vertex position

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle[1]);

		glBindVertexArray(0);
	}
	//------------------------------------------------------------------------------
	void Dome::render() const
	{
		glBindVertexArray(vaoHandle);
		glDrawElements(GL_TRIANGLES, vertices_num, GL_UNSIGNED_INT, ((GLubyte *)NULL + (0)));
	}
	//------------------------------------------------------------------------------

}
