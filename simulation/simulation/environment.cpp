#include "environment.h"

Environment::Environment(char* filename, double thickness, double penetrationCoef, double elasticityCoef, double frictionResistanceCoef, double frictionCoef, double scale, Vec3d basePosition, Mat3d baseOrientation) {
    objMesh = new ObjMesh(filename);
	objMesh->triangulate();
	objMesh->scaleUniformly({ 0,0,0 }, scale);
	objMesh->transformRigidly(basePosition, baseOrientation);

	Vec3d bmin, bmax;
	objMesh->getBoundingBox(1, &bmin, &bmax);
	bmin -= {thickness, thickness, thickness};
	bmax += {thickness, thickness, thickness};
	boundingBox = new BoundingBox(bmin, bmax);

    this->thickness = thickness;
    this->penetrationCoef = penetrationCoef;
    this->elasticityCoef = elasticityCoef;
    this->frictionResistanceCoef = frictionResistanceCoef;
    this->frictionCoef = frictionCoef;

	objMeshRender = new ObjMeshRender(objMesh);
}