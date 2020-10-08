#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <vector>
#include "snake.h"
#include "environment.h"
#include "openGL-headers.h"

namespace py = pybind11;
using namespace std;

Snake* snake;
vector<Environment*> environment;
Vec3d goal = { -DBL_MAX,-DBL_MAX,-DBL_MAX };

Vec3d convertTranslation(py::array_t<double> basePosition) {
	if (basePosition.size() != 3)
		return { 0,0,0 };
	else
		return Vec3d((double*)basePosition.request().ptr);
}
Mat3d convertRotation(py::array_t<double> baseOrientation) {
	if (baseOrientation.size() != 3) 
		return Mat3d::Identity;
	else {
		Vec3d axis = Vec3d((double*)baseOrientation.request().ptr);
		double theta = len(axis);
		if (theta != 0)
			axis.normalize();

		return Mat3d::Identity * cos(theta) + tensorProduct(axis, axis) * (1 - cos(theta)) + Mat3d({ 0,-axis[2],axis[1],axis[2],0,-axis[0],-axis[1],axis[0],0 }) * sin(theta);
	}
}

void loadSnake(char* filename, double timestep, double dampingMassCoef, double dampingStiffnessCoef, double gravity, double muscleStiffnessCoef, py::array_t<double> basePosition, py::array_t<double> baseOrientation) {
	snake = new Snake(filename, timestep, dampingMassCoef, dampingStiffnessCoef, gravity, muscleStiffnessCoef, convertTranslation(basePosition), convertRotation(baseOrientation));

	for (int i = 0; i < environment.size(); i++)
		snake->collision->addEnvironment(environment[i]->objMesh, environment[i]->boundingBox, environment[i]->thickness, environment[i]->penetrationCoef, environment[i]->elasticityCoef, environment[i]->frictionResistanceCoef, environment[i]->frictionCoef);
}
void loadEnvironment(char* filename, double thickness, double penetrationCoef, double elasticityCoef, double frictionResistanceCoef, double frictionCoef, double scale, py::array_t<double> basePosition, py::array_t<double> baseOrientation) {
	environment.push_back(new Environment(filename, thickness, penetrationCoef, elasticityCoef, frictionResistanceCoef, frictionCoef, scale, convertTranslation(basePosition), convertRotation(baseOrientation)));
}
void setGoal(double x, double y, double z) { goal = { x,y,z }; }

int getNumVertex() { return snake->tetMesh->getNumVertices(); }
py::array_t<double> getVertexPosition() { return py::array_t<double>({ getNumVertex(), 3 }, { 3 * sizeof(double), sizeof(double) }, snake->tetMesh->getVertices()->data()); }
py::array_t<double> getVertexVelocity() { return py::array_t<double>({ getNumVertex(), 3 }, { 3 * sizeof(double), sizeof(double) }, snake->velocity->data()); }
int getNumCube() { return snake->tetMesh->getNumVertices() / 10; }
py::array_t<double> getCubePosition() { return py::array_t<double>({ getNumCube(), 3 }, { 30 * sizeof(double), sizeof(double) }, snake->tetMesh->getVertex(7).data()); }
py::array_t<double> getCubeVelocity() { return py::array_t<double>({ getNumCube(), 3 }, { 30 * sizeof(double), sizeof(double) }, snake->velocity[7].data()); }
int getNumMuscle() { return snake->muscle->muscleSegment.size(); }
py::array_t<double> getMuscleLength() { return py::array_t<double>({ getNumMuscle() }, { sizeof(double) }, snake->muscle->getMuscleLength()); }
py::array_t<double> getInverseLocalFrame() { return py::array_t<double>({ 3, 3 }, { 3 * sizeof(double), sizeof(double) }, snake->inverseLocalFrame.data()); }

void contractMuscle(py::array_t<double> contractRate) { snake->muscle->contractMuscle((double*)contractRate.request().ptr); }
void resetSimulation() { snake->resetSimulation(); }
void pauseSimulation() { snake->pauseSimulation(); }
void simulate() { snake->simulate(); }

void drawGlobalFrame(bool GLOBALFRAME) {
	if (GLOBALFRAME) {
		glBegin(GL_LINES);
		glColor3d(1., 0., 0.);
		glVertex3d(0., 0., 0.);
		glVertex3d(1., 0., 0.);
		glColor3d(0., 1., 0.);
		glVertex3d(0., 0., 0.);
		glVertex3d(0., 1., 0.);
		glColor3d(0., 0., 1.);
		glVertex3d(0., 0., 0.);
		glVertex3d(0., 0., 1.);
		glEnd();
	}
}
void drawGrid(bool GRID) {
	if (GRID) {
		int size = 5;

		glBegin(GL_LINES);
		glColor3d(0.75, 0.75, 0.75);
		for (int i = -size;i <= size;i++) {
			glVertex3d(i, 0, -size);
			glVertex3d(i, 0, size);
			glVertex3d(-size, 0, i);
			glVertex3d(size, 0, i);
		}
		glEnd();
	}
}
void drawLocalFrame(bool LOCALFRAME) {
	if (LOCALFRAME) {
		Vec3d& center = snake->tetMesh->getVertex(7);
		Vec3d& xAxis = snake->localFrame[0];
		Vec3d& yAxis = snake->localFrame[1];
		Vec3d& zAxis = snake->localFrame[2];

		glBegin(GL_LINES);
		glColor3d(1., 0., 0.);
		glVertex3d(center[0], center[1], center[2]);
		glVertex3d(center[0] + xAxis[0], center[1] + xAxis[1], center[2] + xAxis[2]);
		glColor3d(0., 1., 0.);
		glVertex3d(center[0], center[1], center[2]);
		glVertex3d(center[0] + yAxis[0], center[1] + yAxis[1], center[2] + yAxis[2]);
		glColor3d(0., 0., 1.);
		glVertex3d(center[0], center[1], center[2]);
		glVertex3d(center[0] + zAxis[0], center[1] + zAxis[1], center[2] + zAxis[2]);
		glEnd();
	}
}
void drawDirection(bool DIRECTION) {
	if (DIRECTION) {
		Vec3d& center = snake->tetMesh->getVertex(7);
		Vec3d frontDirection = norm(snake->tetMesh->getVertex(2) - center);
		Vec3d upDirection = norm(snake->tetMesh->getVertex(5) - center);
		Vec3d goalDirection = norm(goal - center);

		glBegin(GL_LINES);
		glColor3d(1., 0., 0.);
		glVertex3d(center[0], center[1], center[2]);
		glVertex3d(center[0] + frontDirection[0], center[1] + frontDirection[1], center[2] + frontDirection[2]);
		glColor3d(0., 1., 0.);
		glVertex3d(center[0], center[1], center[2]);
		glVertex3d(center[0] + upDirection[0], center[1] + upDirection[1], center[2] + upDirection[2]);
		glColor3d(0., 0., 1.);
		glVertex3d(center[0], center[1], center[2]);
		glVertex3d(center[0] + goalDirection[0], center[1] + goalDirection[1], center[2] + goalDirection[2]);
		glEnd();
	}
}
void drawVelocity(bool VELOCITY) {
	if (VELOCITY) {
		Vec3d& center = snake->tetMesh->getVertex(7);
		Vec3d& velocity = snake->velocity[7];

		glBegin(GL_LINES);
		glColor3d(0.5, 0.5, 0.5);
		glVertex3d(center[0], center[1], center[2]);
		glVertex3d(center[0] + velocity[0], center[1] + velocity[1], center[2] + velocity[2]);
		glEnd();
	}
}
void drawSnake(bool WIRE, bool SOLID) {
	if (WIRE) {
		glColor3d(0, 1, 0);
		snake->renderVolumetricMesh->RenderWireframe(snake->tetMesh);
	}
	if (SOLID) {
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(2., 1.);
		glColor3d(0.5, 1, 0.5);
		snake->renderVolumetricMesh->Render(snake->tetMesh);
		glDisable(GL_POLYGON_OFFSET_FILL);
	}
}
void drawEnvironment(bool WIRE, bool SOLID) {
	if (WIRE) {
		glColor3d(0, 0, 0);
		for (int i = 0; i < environment.size(); i++)
			environment[i]->objMeshRender->render(OBJMESHRENDER_EDGES, OBJMESHRENDER_NONE);
	}
	if (SOLID) {
		float lightPos[4] = { 0,1,0,1 };
		float lightColor[4] = { 1,1,1,1 };

		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
		glLightfv(GL_LIGHT0, GL_AMBIENT, lightColor);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
		glLightfv(GL_LIGHT0, GL_SPECULAR, lightColor);
		for (int i = 0; i < environment.size(); i++)
			environment[i]->objMeshRender->render(OBJMESHRENDER_TRIANGLES, OBJMESHRENDER_SMOOTH | OBJMESHRENDER_MATERIAL);
		glDisable(GL_LIGHTING);
	}
}
void drawMuscle(bool MUSCLE) {
	if (MUSCLE) {
		glBegin(GL_LINES);
		glColor3d(0, 0, 1);
		for (int i = 0; i < snake->muscle->muscleSegment.size(); i++) {
			Vec3d& vertex1 = snake->tetMesh->getVertex(snake->muscle->muscleSegment[i].index1);
			Vec3d& vertex2 = snake->tetMesh->getVertex(snake->muscle->muscleSegment[i].index2);
			Vec3d& vertex3 = snake->tetMesh->getVertex(snake->muscle->muscleSegment[i].index3);

			glVertex3d(vertex1[0], vertex1[1], vertex1[2]);
			glVertex3d(vertex2[0], vertex2[1], vertex2[2]);
			glVertex3d(vertex2[0], vertex2[1], vertex2[2]);
			glVertex3d(vertex3[0], vertex3[1], vertex3[2]);
		}
		glEnd();
	}
}
void drawBoundingBox(bool BOUNDINGBOX) {
	if (BOUNDINGBOX) {
		glColor3d(1, 0, 0);
		snake->tetMesh->getBoundingBox().render();
		for (int i = 0; i < environment.size(); i++)
			environment[i]->boundingBox->render();
	}
}
void drawGoal(bool GOAL) {
	if (GOAL) {
		double size = 0.2;

		glColor3d(1., 1., 0.);
		glBegin(GL_TRIANGLE_FAN);
		glVertex3d(goal[0], goal[1], goal[2]);
		glVertex3d(goal[0] + size / 2, goal[1] + size, goal[2] + size / 2);
		glVertex3d(goal[0] - size / 2, goal[1] + size, goal[2] + size / 2);
		glVertex3d(goal[0] - size / 2, goal[1] + size, goal[2] - size / 2);
		glVertex3d(goal[0] + size / 2, goal[1] + size, goal[2] - size / 2);
		glVertex3d(goal[0] + size / 2, goal[1] + size, goal[2] + size / 2);
		glEnd();
	}
}

PYBIND11_MODULE(simulation, m) {
	m.def("loadSnake", &loadSnake, py::arg("filename") = "", py::arg("timestep") = 0.001, py::arg("dampingMassCoef") = 0, py::arg("dampingStiffnessCoef") = 0, py::arg("gravity") = 0, py::arg("muscleStiffnessCoef") = 0, py::arg("basePosition") = py::array_t<double>(), py::arg("baseOrientation") = py::array_t<double>());
	m.def("loadEnvironment", &loadEnvironment, py::arg("filename") = "", py::arg("thickness") = 0, py::arg("penetrationCoef") = 0, py::arg("elasticityCoef") = 0, py::arg("frictionResistanceCoef") = 0, py::arg("frictionCoef") = 0, py::arg("scale") = 1, py::arg("basePosition") = py::array_t<double>(), py::arg("baseOrientation") = py::array_t<double>());
	m.def("setGoal", &setGoal);

	m.def("getNumVertex", &getNumVertex);
	m.def("getVertexPosition", &getVertexPosition);
	m.def("getVertexVelocity", &getVertexVelocity);
	m.def("getNumCube", &getNumCube);
	m.def("getCubePosition", &getCubePosition);
	m.def("getCubeVelocity", &getCubeVelocity);
	m.def("getNumMuscle", &getNumMuscle);
	m.def("contractMuscle", &contractMuscle);

	m.def("getMuscleLength", &getMuscleLength);
	m.def("resetSimulation", &resetSimulation);
	m.def("pauseSimulation", &pauseSimulation);
	m.def("simulate", &simulate);

	m.def("drawGlobalFrame", &drawGlobalFrame);
	m.def("drawGrid", &drawGrid);
	m.def("drawLocalFrame", &drawLocalFrame);
	m.def("drawDirection", &drawDirection);
	m.def("drawVelocity", &drawVelocity);
	m.def("drawSnake", &drawSnake);
	m.def("drawEnvironment", &drawEnvironment);
	m.def("drawMuscle", &drawMuscle);
	m.def("drawBoundingBox", &drawBoundingBox);
	m.def("drawGoal", &drawGoal);
}