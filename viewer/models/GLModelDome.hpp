#pragma once

#include "drawable.h"
#include <string>
#include <vector>

class GLModelDome : public Drawable
{
private:
    unsigned int vaoHandle;
	int vertices_num;

public:
	GLModelDome(float radius, int divide_num);

    void render() const;

private:

};

