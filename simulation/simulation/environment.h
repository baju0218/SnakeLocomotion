#pragma once
#include "objMesh.h"
#include "boundingBox.h"
#include "objMeshRender.h"

class Environment {
public:
	Environment(char* filename, double thickness, double penetrationCoef, double elasticityCoef, double frictionResistanceCoef, double frictionCoef, double scale, Vec3d basePosition, Mat3d baseOrientation);

	/* Attribute */
	ObjMesh* objMesh;
	BoundingBox* boundingBox;

	double thickness;
	double penetrationCoef;
	double elasticityCoef;
	double frictionResistanceCoef;
	double frictionCoef;

	ObjMeshRender* objMeshRender;
};