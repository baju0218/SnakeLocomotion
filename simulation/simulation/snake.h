#pragma once
#include "volumetricMeshLoader.h"
#include "volumetricMesh.h"
#include "tetMesh.h"
#include "corotationalLinearFEM.h"
#include "corotationalLinearFEMStencilForceModel.h"
#include "forceModelAssembler.h"
#include "generateMassMatrix.h"
#include "implicitBackwardEulerSparse.h"
#include "muscle.h"
#include "collision.h"
#include "renderVolumetricMesh.h"

class Snake {
public:
	Snake(char* filename, double timestep, double dampingMassCoef, double dampingStiffnessCoef, double gravity, double muscleStiffnessCoef, Vec3d basePosition, Mat3d baseOrientation);

	void resetSimulation();
	void pauseSimulation();
	void simulate();

	/* Attribute */
	TetMesh* tetMesh;
	CorotationalLinearFEM* corotationalLinearFEM;
	StencilForceModel* stencilForceModel;
	ForceModel* forceModel;
	SparseMatrix* massMatrix;
	ImplicitBackwardEulerSparse* implicitBackwardEulerSparse;

	double* mass;
	Vec3d* deformation;
	Vec3d* velocity;
	Vec3d* force;

	Vec3d* gravityForce;
	Vec3d* muscleForce;
	Vec3d* collisionForce;

	Muscle* muscle;
	Collision* collision;

	Mat3d localFrame;
	Mat3d inverseLocalFrame;

	bool isSimulate;

	RenderVolumetricMesh* renderVolumetricMesh;
};