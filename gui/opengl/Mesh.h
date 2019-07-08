/*
 * Mesh.h
 *
 *  Created on: Jun 26, 2015
 *      Author: gdurandau
 */

#ifndef MESH_H_
#define MESH_H_

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

#include <DrawingUtility.h>

using namespace std;

class Mesh {
public:
	Mesh() {}
	Mesh(vector<float>& vertices, vector<float>& normals, vector<GLushort>& faces);
	virtual ~Mesh();

	void draw(short representation) const;
	void getBoundingSphere(float& radius, fVec3& center);

private:
    int numVertices;
    GLuint vertBuffer, normBuffer;
    vector<GLushort> edges, faces;
    Vec3 center;
    float radius;
};

#endif /* MESH_H_ */
