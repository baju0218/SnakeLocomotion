#include "collision.h"

Collision::Collision(TetMesh* tetMesh, double timestep, double* mass, Vec3d* velocity, Vec3d* force, Vec3d* collisionForce) {
	this->tetMesh = tetMesh;
	this->timestep = timestep;
	this->mass = mass;
	this->velocity = velocity;
	this->force = force;
	this->collisionForce = collisionForce;
}

void Collision::addEnvironment(ObjMesh* objMesh, BoundingBox* boundingBox, double thickness, double penetrationCoef, double elasticityCoef, double frictionResistanceCoef, double frictionCoef) {
	this->objMesh.push_back(objMesh);
	this->boundingBox.push_back(boundingBox);
	this->thickness.push_back(thickness);
	this->penetrationCoef.push_back(penetrationCoef);
	this->elasticityCoef.push_back(elasticityCoef);
	this->frictionResistanceCoef.push_back(frictionResistanceCoef);
	this->frictionCoef.push_back(frictionCoef);
}

void Collision::detectCollision() {
	BoundingBox snakeBound = tetMesh->getBoundingBox();

	/* Environment */
	for (int envIndex = 0; envIndex < objMesh.size(); envIndex++) {

		/* Intersect */
		if (snakeBound.intersect(*boundingBox[envIndex])) {

			/* Vertex */
			for (int i = 0; i < tetMesh->getNumVertices(); i++) {
				Vec3d X = tetMesh->getVertex(i);
				Vec3d V = velocity[i];
				Vec3d F = force[i];

				bool isInside = true;
				ObjMesh::Face nearestFace;
				double nearestDistance = -DBL_MAX;

				/* Group */
				for (int groupIndex = 0; groupIndex < objMesh[envIndex]->getNumGroups() && isInside; groupIndex++) {
					ObjMesh::Group group = objMesh[envIndex]->getGroup(groupIndex);

					/* Face */
					for (int faceIndex = 0; faceIndex < group.getNumFaces() && isInside; faceIndex++) {
						ObjMesh::Face face = group.getFace(faceIndex);
						
						Vec3d P[3], N;
						for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
							P[vertexIndex] = objMesh[envIndex]->getPosition(face, vertexIndex);
						N = norm(cross(P[1] - P[0], P[2] - P[0]));

						double distance = dot(X - P[0], N);
						if (distance > thickness[envIndex])
							isInside = false;
						else if (distance > nearestDistance) {
								nearestFace = face;
								nearestDistance = distance;
						}
					}
				}

				/* Detect */
				if (isInside) {
					Vec3d P[3], N;
					for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
						P[vertexIndex] = objMesh[envIndex]->getPosition(nearestFace, vertexIndex);
					N = norm(cross(P[1] - P[0], P[2] - P[0]));

					Vec3d Vnor = dot(N, V) * N;
					Vec3d Vtan = V - Vnor;

					Vec3d Fnor = dot(N, F) * N;
					Vec3d Ftan = F - Fnor;

					// Collision
					if (nearestDistance < thickness[envIndex])
						collisionForce[i] -= mass[i] * penetrationCoef[envIndex] * (nearestDistance - thickness[envIndex]) * N / (timestep * timestep);
					if (dot(N, V) < 0)
						collisionForce[i] -= mass[i] * (1 + elasticityCoef[envIndex]) * Vnor / timestep;

					// Contact
					if (dot(N, V) < thickness[envIndex])
						collisionForce[i] -= mass[i] * frictionResistanceCoef[envIndex] * Vtan;
					if (dot(N, F) < 0) {
						if (len(Ftan) > frictionCoef[envIndex] * len(Fnor))
							collisionForce[i] -= frictionCoef[envIndex] * len(Fnor) * norm(Ftan);
						else
							collisionForce[i] -= Ftan;
					}
				}
			}
		}
	}
}