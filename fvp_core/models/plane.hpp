#pragma once

#include "models/drawable.hpp"
namespace model {
	class Plane : public Drawable
	{
	private:
		unsigned int vaoHandle;
		int faces;

	public:
		Plane(float, float, int, int, float smax = 1.0f, float tmax = 1.0f);

		void render() const;
	};
}
