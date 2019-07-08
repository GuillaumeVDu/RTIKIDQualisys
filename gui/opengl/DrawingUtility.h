#ifndef DRAWINGUTILITY_H_
#define DRAWINGUTILITY_H_

#include "SimTKcommon.h"

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include <GL/glut.h>

#include <cstdlib>
#include <string>
#include <algorithm>
#include <set>
#include <vector>
#include <utility>
#include <limits>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <sys/stat.h>

using namespace SimTK;
using namespace std;

static void computeBoundingSphereForVertices(const vector<float>& vertices,
		float& radius, Vec3& center)
{
	fVec3 lower(vertices[0], vertices[1], vertices[2]);
	fVec3 upper = lower;
	for (int i = 3; i < (int) vertices.size(); i += 3)
	{
		for (int j = 0; j < 3; j++)
		{
			lower[j] = min(lower[j], vertices[i + j]);
			upper[j] = max(upper[j], vertices[i + j]);
		}
	}
	center = (lower + upper) / 2;
	float rad2 = 0;
	for (int i = 0; i < (int) vertices.size(); i += 3)
	{
		float x = center[0] - vertices[i];
		float y = center[1] - vertices[i + 1];
		float z = center[2] - vertices[i + 2];
		float norm2 = x * x + y * y + z * z;
		if (norm2 > rad2)
			rad2 = norm2;
	}
	radius = sqrt(rad2);
}



//static void drawSkyVertex(fVec3 position, float texture)
//{
//	glTexCoord1f(texture);
//	glVertex3fv(&position[0]);
//}

#endif
