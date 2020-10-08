#pragma once
#include "tetMesh.h"
#include "objMesh.h"

using namespace std;

class Collision {
public:
	Collision(TetMesh* tetMesh, double timestep, double* mass, Vec3d* velocity, Vec3d* force, Vec3d* conllisionForce);
	void addEnvironment(ObjMesh* objMesh, BoundingBox* boundingBox, double thickness, double penetrationCoef, double elasticityCoef, double frictionResistanceCoef, double frictionCoef);

	void detectCollision();

	/* Attribute */
	TetMesh* tetMesh;
	double timestep;
	double* mass;
	Vec3d* velocity;
	Vec3d* force;
	Vec3d* collisionForce;

	vector<ObjMesh*> objMesh;
	vector<BoundingBox*> boundingBox;
	vector<double> thickness;
	vector<double> penetrationCoef;
	vector<double> elasticityCoef;
	vector<double> frictionResistanceCoef;
	vector<double> frictionCoef;
};