#include "muscle.h"

Muscle::Muscle(TetMesh* tetMesh, double* mass, double muscleStiffnessCoef, Vec3d* muscleForce) {
	this->tetMesh = tetMesh;
	this->mass = mass;
	this->muscleStiffnessCoef = muscleStiffnessCoef;
	this->muscleForce = muscleForce;

	int numCube = tetMesh->getNumVertices() / 10;
	int index[24] = { 0,5,11,1,5,10, 3,6,10,0,6,13, 4,9,13,3,9,14, 1,8,14,4,8,11 };
	for (int i = 0; i < numCube; i++) {
		for (int j = 0; j < 8; j++) {
			int index1 = 10 * i + index[j * 3];
			int index2 = 10 * i + index[j * 3 + 1];
			int index3 = 10 * i + index[j * 3 + 2];
			double length1 = len(tetMesh->getVertex(index1) - tetMesh->getVertex(index2));
			double length2 = len(tetMesh->getVertex(index2) - tetMesh->getVertex(index3));

			muscleSegment.push_back({ index1,index2,index3,length1,length2 });
		}
	}
	this->muscleLength = new double[8 * numCube];
}

double* Muscle::getMuscleLength() {
	for (int i = 0; i < muscleSegment.size(); i++) {
		int index1 = muscleSegment[i].index1;
		int index2 = muscleSegment[i].index2;
		int index3 = muscleSegment[i].index3;
		double currentLength1 = len(tetMesh->getVertex(index1) - tetMesh->getVertex(index2));
		double currentLength2 = len(tetMesh->getVertex(index2) - tetMesh->getVertex(index3));

		muscleLength[i] = currentLength1 + currentLength2;
	}

	return muscleLength;
}

void Muscle::contractMuscle(double* contractRate) {
	for (int i = 0; i < muscleSegment.size(); i++) {
		int index1 = muscleSegment[i].index1;
		int index2 = muscleSegment[i].index2;
		int index3 = muscleSegment[i].index3;
		double length1 = muscleSegment[i].length1;
		double length2 = muscleSegment[i].length2;

		double currentLength1 = len(tetMesh->getVertex(index1) - tetMesh->getVertex(index2));
		double desiredLength1 = contractRate[i] * length1;
		if (currentLength1 > desiredLength1) {
			muscleForce[index1] -= mass[index1] * muscleStiffnessCoef * (currentLength1 - desiredLength1) * norm(tetMesh->getVertex(index1) - tetMesh->getVertex(index2));
			muscleForce[index2] -= mass[index2] * muscleStiffnessCoef * (currentLength1 - desiredLength1) * norm(tetMesh->getVertex(index2) - tetMesh->getVertex(index1));
		}

		double currentLength2 = len(tetMesh->getVertex(index2) - tetMesh->getVertex(index3));
		double desiredLength2 = contractRate[i] * length2;
		if (currentLength2 > desiredLength2) {
			muscleForce[index2] -= mass[index2] * muscleStiffnessCoef * (currentLength2 - desiredLength2) * norm(tetMesh->getVertex(index2) - tetMesh->getVertex(index3));
			muscleForce[index3] -= mass[index3] * muscleStiffnessCoef * (currentLength2 - desiredLength2) * norm(tetMesh->getVertex(index3) - tetMesh->getVertex(index2));
		}
	}
}