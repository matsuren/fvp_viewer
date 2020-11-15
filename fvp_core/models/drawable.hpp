#pragma once

namespace model {
	class Drawable
	{
	public:
		Drawable();

		virtual void render() const = 0;
	};
}
