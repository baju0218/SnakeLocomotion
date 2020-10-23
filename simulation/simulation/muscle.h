#pragma once
#include "tetMesh.h"

using namespace std;

typedef struct MuscleSegment {
	int index1;
	int index2;
	int index3;
	double length1;
	double length2;
};

class Muscle {
public:
	Muscle(TetMesh* tetMesh, double* mass, double muscleStiffnessCoef, Vec3d* muscleForce);

	void contractMuscle(double* contractRate);

	/* Attribute */
	TetMesh* tetMesh;
	double* mass;
	double muscleStiffnessCoef;
	Vec3d* muscleForce;

	double* muscleLength;

	vector<MuscleSegment> muscleSegment;
};