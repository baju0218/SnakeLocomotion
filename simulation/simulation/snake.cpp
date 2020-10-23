#include "snake.h"

Snake::Snake(char* filename, double timestep, double dampingMassCoef, double dampingStiffnessCoef, double gravity, double muscleStiffnessCoef, Vec3d basePosition, Mat3d baseOrientation) {
	VolumetricMesh* volumetricMesh = VolumetricMeshLoader::load(filename);
	tetMesh = dynamic_cast<TetMesh*>(volumetricMesh);
	tetMesh->applyLinearTransformation(basePosition.data(), baseOrientation.data());
	corotationalLinearFEM = new CorotationalLinearFEM(tetMesh);
	stencilForceModel = new CorotationalLinearFEMStencilForceModel(corotationalLinearFEM);
	forceModel = new ForceModelAssembler(stencilForceModel);
	GenerateMassMatrix::computeMassMatrix(tetMesh, &massMatrix, true);
	implicitBackwardEulerSparse = new ImplicitBackwardEulerSparse(3 * tetMesh->getNumVertices(), timestep, massMatrix, forceModel, 0, {}, dampingMassCoef, dampingStiffnessCoef);

	mass = new double[tetMesh->getNumVertices()];
	deformation = new Vec3d[tetMesh->getNumVertices()];
	memset(deformation, 0, sizeof(Vec3d) * tetMesh->getNumVertices());
	velocity = new Vec3d[tetMesh->getNumVertices()];
	memset(velocity, 0, sizeof(Vec3d) * tetMesh->getNumVertices());
	force = new Vec3d[tetMesh->getNumVertices()];
	memset(force, 0, sizeof(Vec3d) * tetMesh->getNumVertices());

	gravityForce = new Vec3d[tetMesh->getNumVertices()];
	tetMesh->computeGravity(gravityForce->data(), gravity);
	for (int i = 0; i < tetMesh->getNumVertices(); i++)
		mass[i] = -gravityForce[i][1] / gravity;
	collisionForce = new Vec3d[tetMesh->getNumVertices()];
	memset(collisionForce, 0, sizeof(Vec3d) * tetMesh->getNumVertices());
	muscleForce = new Vec3d[tetMesh->getNumVertices()];
	memset(muscleForce, 0, sizeof(Vec3d) * tetMesh->getNumVertices());

	muscle = new Muscle(tetMesh, mass, muscleStiffnessCoef, muscleForce);
	collision = new Collision(tetMesh, timestep, mass, velocity, force, collisionForce);

	Vec3d xAxis = norm(tetMesh->getVertex(2) - tetMesh->getVertex(7));
	Vec3d zAxis = norm(cross(xAxis, norm(tetMesh->getVertex(5) - tetMesh->getVertex(7))));
	Vec3d yAxis = norm(cross(zAxis, xAxis));
	localFrame = Mat3d(xAxis, yAxis, zAxis);
	inverseLocalFrame = inv(trans(localFrame));

	isSimulate = true;

	renderVolumetricMesh = new RenderVolumetricMesh();
}

void Snake::resetSimulation() {
	/* Return Deformation */
	for (int i = 0; i < tetMesh->getNumVertices(); i++)
		deformation[i] *= -1;
	tetMesh->applyDeformation(deformation->data());

	/* Reset */
	implicitBackwardEulerSparse->ResetToRest();

	memset(deformation, 0, sizeof(Vec3d) * tetMesh->getNumVertices());
	memset(velocity, 0, sizeof(Vec3d) * tetMesh->getNumVertices());
	memset(force, 0, sizeof(Vec3d) * tetMesh->getNumVertices());
	memset(collisionForce, 0, sizeof(Vec3d) * tetMesh->getNumVertices());
	memset(muscleForce, 0, sizeof(Vec3d) * tetMesh->getNumVertices());

	/* Local Frame */
	Vec3d xAxis = norm(tetMesh->getVertex(2) - tetMesh->getVertex(7));
	Vec3d zAxis = norm(cross(xAxis, norm(tetMesh->getVertex(5) - tetMesh->getVertex(7))));
	Vec3d yAxis = norm(cross(zAxis, xAxis));
	localFrame = Mat3d(xAxis, yAxis, zAxis);
	inverseLocalFrame = inv(trans(localFrame));
}

void Snake::pauseSimulation() {
	isSimulate = !isSimulate;
}

void Snake::simulate() {
	if (!isSimulate)
		return;

	/* Gravity Force */
	implicitBackwardEulerSparse->SetExternalForces(gravityForce->data());

	/* Muscle Force */
	implicitBackwardEulerSparse->AddExternalForces(muscleForce->data());
	memset(muscleForce->data(), 0, sizeof(Vec3d) * tetMesh->getNumVertices());
	
	/* Collision Force */
	collision->detectCollision();
	implicitBackwardEulerSparse->AddExternalForces(collisionForce->data());
	memset(collisionForce->data(), 0, sizeof(Vec3d) * tetMesh->getNumVertices());



	/* Return Deformation */
	for (int i = 0; i < tetMesh->getNumVertices(); i++)
		deformation[i] *= -1;
	tetMesh->applyDeformation(deformation->data());

	/* To calculate Force */
	memcpy(force->data(), velocity->data(), sizeof(Vec3d) * tetMesh->getNumVertices());

	/* Step Simulation */
	implicitBackwardEulerSparse->DoTimestep();
	implicitBackwardEulerSparse->GetqState(deformation->data(), velocity->data());

	/* Calculate force */
	for (int i = 0; i < tetMesh->getNumVertices(); i++)
		force[i] = mass[i] * (velocity[i] - force[i]) / implicitBackwardEulerSparse->GetTimeStep();

	/* Apply Deformation */
	tetMesh->applyDeformation(deformation->data());



	/* Local Frame */
	Vec3d xAxis = norm(tetMesh->getVertex(2) - tetMesh->getVertex(7));
	Vec3d zAxis = norm(cross(xAxis, norm(tetMesh->getVertex(5) - tetMesh->getVertex(7))));
	Vec3d yAxis = norm(cross(zAxis, xAxis));
	localFrame = Mat3d(xAxis, yAxis, zAxis);
	inverseLocalFrame = inv(trans(localFrame));
}