#pragma once

#include "models/drawable.hpp"
#include <string>
#include <vector>

namespace model {
	class Dome : public Drawable
	{
	private:
		unsigned int vaoHandle;
		int vertices_num;

	public:
		Dome(float radius, int divide_num);

		void render() const;

	private:

	};
}


