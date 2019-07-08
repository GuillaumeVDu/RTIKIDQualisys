/*
 * RenderedMesh.h
 *
 *  Created on: Jun 26, 2015
 *      Author: gdurandau
 */

#ifndef RENDEREDMESH_H_
#define RENDEREDMESH_H_

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
#include "Mesh.h"

class RenderedMesh {
public:
	RenderedMesh() {}
	RenderedMesh(const Transform& transform, const Vec3& scale,
			const Vec4& color, short representation,
			unsigned short meshIndex, unsigned short resolution);
	virtual ~RenderedMesh();
	void draw(bool setColor);

	inline void setMesh(vector<float>& vertices, vector<float>& normals, vector<GLushort>& faces)
	{
		_mesh = Mesh(vertices, normals, faces);
	}

	inline const Transform& getTransform() const
	{
		return transform;
	}

	inline void setTransform(const Transform& transform)
	{
		this->transform = transform;
	}

	void computeBoundingSphere(float& radius, fVec3& center);

private:
    Transform transform;
    Vec3 scale;
    GLfloat color[4];
    Mesh _mesh;
    short representation;
    unsigned short meshIndex, resolution;
};

#endif /* RENDEREDMESH_H_ */
