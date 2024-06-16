#include "MassSpringSystemSimulator.h"
#include <sstream>
#include <iostream>
#include <cmath>


/**
 * This function sets up a system of two mass points connected by a spring
 */
void MassSpringSystemSimulator::setupTrambouline() {
	reset();

	//points
	for (int i = 0; i < 11; i++) {
		for (int j = 0; j < 11; j++) {
			pointIndexMap[i][j] = addMassPoint(Vec3(-1.0f + j * 0.2f, -0.5f, 1.0f - i * 0.2f), Vec3(0.0f, 0.0f, 0.0f), false);
		}
	}

	//corner points fixed
	for (int i = 0; i < 11; i++) {
		pointArr.at(pointIndexMap[0][i]).isFixed = true;
		pointArr.at(pointIndexMap[i][0]).isFixed = true;
		pointArr.at(pointIndexMap[i][10]).isFixed = true;
		pointArr.at(pointIndexMap[10][i]).isFixed = true;
	}

	//springs on the top and the left side
	for (int i = 1; i < 11; i++) {
		addSpring(pointIndexMap[0][i - 1], pointIndexMap[0][i], 0.18f);
		addSpring(pointIndexMap[i - 1][0], pointIndexMap[i][0], 0.18f);
	}

	//other springs and rigid bodies serving as panels of the trambouline
	for (int i = 1; i < 11; i++) {
		for (int j = 1; j < 11; j++) {
			addSpring(pointIndexMap[i][j - 1], pointIndexMap[i][j], 0.18f);
			addSpring(pointIndexMap[i - 1][j], pointIndexMap[i][j], 0.18f);

			MassPoint p0 = pointArr.at(pointIndexMap[i - 1][j - 1]);
			MassPoint p1 = pointArr.at(pointIndexMap[i][j - 1]);
			MassPoint p2 = pointArr.at(pointIndexMap[i][j]);
			MassPoint p3 = pointArr.at(pointIndexMap[i - 1][j]);

			Vec3 centerPoint = (p0.Position + p1.Position + p2.Position + p3.Position) / 4.0f;
			RigidBody body = RigidBody(Vec3(centerPoint.x, centerPoint.y, centerPoint.z), Vec3(0.2f, 0.01f, 0.2f), m_fMassPanels);

			GamePhysics::Mat4 matrix;
			matrix.initTranslation(body.Position.x, body.Position.y, body.Position.z);

			GamePhysics::Mat4 rotation = body.Orientation.getRotMat();

			body.m_scaledObjToWorld = rotation * matrix;
			matrix.initScaling(body.Size.x, body.Size.y, body.Size.z);
			body.m_objToWorld = matrix * body.m_scaledObjToWorld;

			Panel panel = Panel(pointIndexMap[i - 1][j - 1], pointIndexMap[i][j - 1], pointIndexMap[i][j], pointIndexMap[i - 1][j], body);

			panelArr.push_back(panel);
		}
	}
}

/* It returns a string containing the names of the test cases
*
*  @return The string "Demo1, Demo2, Demo3, Demo4"
*/
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Project";
}

/**
 * This function is called once when the UI is initialized. It is used to add variables to the tweak
 * bar
 *
 * the parameter DUC is a pointer to the DrawingUtilitiesClass object.
 */
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	//TwAddVarRW(DUC->g_pTweakBar, "Point Size", TW_TYPE_FLOAT, &m_fPointSize, "min=0.01 step=0.01");
	//TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.0 step=0.2");
	/*TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0.0 step=10.0");
	TwAddVarRW(DUC->g_pTweakBar, "Mass of points", TW_TYPE_FLOAT, &m_fMass, "min=0.0 step=0.005");
	TwAddVarRW(DUC->g_pTweakBar, "Mass of panels", TW_TYPE_FLOAT, &m_fMassPanels, "min=0.1 step=0.5");*/
}

/**
 * It resets the current scene
 */
void MassSpringSystemSimulator::reset()
{
	m_externalForce = Vec3(0, 0, 0);
	pointArr.clear();
	springArr.clear();
	panelArr.clear();
}

/**
 * This function is called every frame, and it's responsible for drawing the points of the mass-spring
 * system
 * 
 */
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	// m_iNumPoints
	for (int i = 0; i < pointArr.size(); i++)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(pointArr[i].Position, Vec3(m_fPointSize, m_fPointSize, m_fPointSize));
	}
	for (int i = 0; i < springArr.size(); i++) {
		Spring spring = springArr.at(i);
		MassPoint p1 = pointArr.at(spring.p1);
		MassPoint p2 = pointArr.at(spring.p2);
		DUC->beginLine();
		DUC->drawLine(p1.Position, Vec3(255.0, 255.0, 255.0), p2.Position, Vec3(125.0, 125.0, 125.0));
		DUC->endLine();
	}

	for (auto it = panelArr.begin(); it != panelArr.end(); it++) {
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, it->color);
		DUC->drawRigidBody(it->m_objToWorld);
	}
}

/**
 * This function is called when the user changes the test case (the demo)

 */
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	setupTrambouline();
}

/*
The function is responsible for applying the mouse deltas to the movable object's position
*/
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Vec3 pullForce(0, 0, 0);
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.5f;
		pullForce = pullForce + inputWorld * inputScale;
		applyExternalForce(pullForce);
	}
}

/**
 * This function is used to simulate the time step of the system
 */
void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	midPointMethodIntegration(timeStep);
	updatePanelArray();
	//BoundingBoxCheck();
}

void MassSpringSystemSimulator::updatePanelArray() {
	int panelIndex = 0;
	for (int i = 1; i < 11 && panelIndex < panelArr.size(); i++) {
		for (int j = 1; j < 11 && panelIndex < panelArr.size(); j++) {
			MassPoint p0 = pointArr.at(pointIndexMap[i - 1][j - 1]);
			MassPoint p1 = pointArr.at(pointIndexMap[i - 1][j]);
			MassPoint p2 = pointArr.at(pointIndexMap[i][j - 1]);
			MassPoint p3 = pointArr.at(pointIndexMap[i][j]);

			//updating position
			Vec3 centerPoint = (p0.Position + p1.Position + p2.Position + p3.Position) / 4.0f;
			Vec3 displacement = centerPoint - panelArr[panelIndex].Position;
			panelArr[panelIndex].Position = Vec3(centerPoint.x, centerPoint.y, centerPoint.z);

			//updating rotation
			Vec3 rotationDifX = p3.Position - p1.Position;
			Vec3 rotationDifZ = p1.Position - p0.Position;
			double rotX = acos(rotationDifX.Z / norm(rotationDifX));
			double rotZ = acos(rotationDifZ.X / norm(rotationDifZ));
			if (p3.Position.Y < p1.Position.Y) rotX *= -1;
			if (p0.Position.Y < p1.Position.Y) rotZ *= -1;
			Quat rot(rotX, 0, rotZ);
			panelArr[panelIndex].Orientation = rot;
			//Vec3 springForceOnPanel = panelArr[panelIndex].Mass * displacement / pow(timeStep, 2);
			//applyForceOnBody(panelIndex, centerPoint, springForceOnPanel);

			panelArr[panelIndex].LinearVelocity = (p0.Velocity + p1.Velocity + p2.Velocity + p3.Velocity) / 4.0f;

			//updating objToWorld matrix for view
			panelArr[panelIndex].updateWorldMatrix();

			panelIndex++;
		}
	}
}

/**
 * This function is called when the user presses the left mouse button down. It sets the m_trackmouse.x
 * and m_trackmouse.y to the current mouse position
 *
 * The parameter x: is the x coordinate of the mouse click
 * The parameter y: is the y coordinate of the mouse click.
 */
void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

/**
 *
 *
 * The parameter mass: is the mass of the particle.
 */
void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

/**
 * The parameter stiffness: is the stiffness of the springs
 */
void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

/**
 * This function sets the damping factor of the system
 *
 * The parameter damping: is the damping factor of the system.
 */
void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

/**
 * This function adds a mass point to the system
 *
 * The parameter position: is the position of the mass point.
 * The parameter Velocity: is the initial velocity of the mass point.
 * The parameter isFixed:  if true, the mass point will not move.
 *
 * @return The index of the last mass point added to the system.
 */
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	MassPoint p = MassPoint(position, Velocity, Vec3(0, 0, 0), isFixed);

	pointArr.push_back(p);

	return getNumberOfMassPoints() - 1;
}

/**
 * The parameter masspoint1: is the index of the first mass point in the mass point array.
 * The parameter masspoint2: is the index of the second mass point.
 * The parameter initialLength: is the initial length of the spring
 */
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring s = Spring(masspoint1, masspoint2, initialLength);
	springArr.push_back(s);
}

/**

 * @return The number of mass points in the system.
 */
int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return pointArr.size();
}

/**
 * @return The number of springs in the system.
 */
int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springArr.size();
}

/**
 * This function adds the force to the external force
 *
 *The parameter force: is the force to be applied to the system.
 */
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{

}

/**
 * The function calculates the midpoint method of integration by first calculating the euler method of
 * integration for half the time step, then calculating the forces and acceleration for the new
 * position and velocity, and finally calculating the new position and velocity

 */
void MassSpringSystemSimulator::midPointMethodIntegration(float timeStep)
{
	//save points to reset their state after midpoint step
	std::vector<MassPoint> originalPoints = pointArr;

	ComputeForces();
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (pointArr[i].isFixed) {
			continue;
		}

		pointArr[i].Acceleration = pointArr[i].Force / m_fMass;
		pointArr[i].Position = pointArr[i].Position + (timeStep / 2.0f * pointArr[i].Velocity);
		pointArr[i].Velocity = pointArr[i].Velocity + (timeStep / 2.0f * pointArr[i].Acceleration);
	}

	ComputeForces();
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (pointArr[i].isFixed) {
			continue;
		}

		pointArr[i].Acceleration = pointArr[i].Force / m_fMass;
		pointArr[i].Position = originalPoints[i].Position + (timeStep * pointArr[i].Velocity);
		pointArr[i].Velocity = originalPoints[i].Velocity + (timeStep * pointArr[i].Acceleration);
	}
}

void MassSpringSystemSimulator::ComputeForces() {
	
	Vec3 gravity = Vec3(0.0, -9.81f, 0.0);

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].Force = Vec3(0, 0, 0);
		pointArr[i].Force += gravity * m_fMass;
		//pointArr[i].Force += m_externalForce;

		Vec3 vel = pointArr[i].Velocity;

		pointArr[i].Force += vel * -m_fDamping;
	}

	for (int i = 0; i < getNumberOfSprings(); i++) {
		MassPoint p1 = pointArr[springArr[i].p1];
		MassPoint p2 = pointArr[springArr[i].p2];

		Vec3 dist = p1.Position - p2.Position;
		float length = norm(dist);
		Vec3 direction = dist / length;

		Vec3 springForce = -m_fStiffness * (length - springArr[i].initialLength) * direction;

		pointArr[springArr[i].p1].Force += springForce;
		pointArr[springArr[i].p2].Force -= springForce;
	}
}

void MassSpringSystemSimulator::BoundingBoxCheck() {
	for (size_t i = 0; i < pointArr.size(); i++)
	{
		if (!pointArr[i].isFixed)
		{
			Vec3 pos = pointArr[i].Position;
			Vec3 vel = pointArr[i].Velocity;

			for (int f = 0; f < 6; f++)
			{
				float sign = (f % 2 == 0) ? -1.0f : 1.0f;
				if (sign * pos.value[f / 2] < -0.5f)
				{
					pos.value[f / 2] = sign * -0.5f;
					vel.value[f / 2] = 0;
				}
			}

			pointArr[i].Position = pos;
			pointArr[i].Velocity = vel;
		}
	}
}

