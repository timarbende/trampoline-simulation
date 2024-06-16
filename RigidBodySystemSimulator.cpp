#include "RigidBodySystemSimulator.h"
#include <sstream>
#include <iostream>
#include <cmath>
#include "collisionDetect.h"

/**
 * This function adds the force to the external force
 *
 * @param force The force to be applied to the system.
 */
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	rigidArr[i].appliedForces.push_back(Force{ loc, force });
}


/**
 * This function adds a rigid body to the system
 */
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody body = RigidBody(position, size, mass);

	rigidArr.push_back(body);
}

/**
 * This function sets the orientation of the ith rigid body
 */
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidArr[i].Orientation = orientation;
	rigidArr[i].updateWorldMatrix();
}

/**
 * This function sets the linear velocity of the ith rigid body
 */
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidArr[i].LinearVelocity = velocity;
}

void RigidBodySystemSimulator::torqueIntegrator(RigidBody& body) {
	Vec3 cross_P = Vec3(0.f, 0.f, 0.f);

	for (int i = 0; i < body.appliedForces.size(); i++) {
		Force force = body.appliedForces[i];
		cross_P += cross(force.location, force.direction);
	}

	body.Torque = cross_P;
}

void RigidBodySystemSimulator::orientationIntegrator(RigidBody& body, float timestep)
{
	body.Orientation += Quat(body.AngularVelocity.x,body.AngularVelocity.y,body.AngularVelocity.z, 0) * body.Orientation * (timestep / 2.0f);
	body.Orientation = body.Orientation.unit();
}

void RigidBodySystemSimulator::positionIntegrator(RigidBody& body, float timestep)
{
	body.Position += body.LinearVelocity * timestep;
}

void RigidBodySystemSimulator::angularMomentumIntegrator(RigidBody& body, float timestep)
{
	body.AngularMomentum += timestep * body.Torque;
}

std::vector<std::vector<float>> RigidBodySystemSimulator::inverseMatrix3x3(std::vector<std::vector<float>> m)
{
	double det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
		m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
		m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

	double invdet = 1 / det;

	std::vector<std::vector<float>> inverse(3, std::vector<float>(3, 0)); // inverse of matrix m
	inverse[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
	inverse[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
	inverse[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
	inverse[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
	inverse[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
	inverse[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
	inverse[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
	inverse[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
	inverse[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;

	return inverse;
}

std::vector<std::vector<float>> RigidBodySystemSimulator::transposeMatrix3x3(std::vector<std::vector<float>> vec)
{
	std::vector<std::vector<float>> transpose(3, std::vector<float>(3, 0));
	for (int i = 0; i < 9; i++)
	{
		int r = i / 3;
		int c = i % 3;
		transpose[r][c] = vec[c][r];
	}
	return transpose;

}

void RigidBodySystemSimulator::inertiaTensorIntegrator(RigidBody& body)
{
	matrix4x4<double> rotMat = body.Orientation.getRotMat();
	matrix4x4<double> rotMatTransposed = rotMat;
	rotMatTransposed.transpose();

	body.invertedInertiaTensor = rotMat * body.invertedInitialInertiaTensor * rotMatTransposed;

	/*std::cout << "Updated (inverse) Inertia Tensor " << std::endl;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			std::cout << rigidArr[index].invertedInitialInertiaTensor.value[i][j] << "\t";
		}
		std::cout << std::endl;
	}*/
}

void RigidBodySystemSimulator::angularVelocityIntegrator(RigidBody& body)
{
	body.AngularVelocity = body.invertedInertiaTensor * body.AngularMomentum;
}

void RigidBodySystemSimulator::linearVelocityIntegrator(RigidBody& body, float timestep)
{
	Vec3 acceleration = Vec3(0, 0, 0);
	for (int i = 0; i < body.appliedForces.size(); i++) {
		acceleration += body.appliedForces[i].direction;
	}
	acceleration /= body.Mass;

	body.LinearVelocity += acceleration * timestep;
}

void RigidBodySystemSimulator::linearVelocityIntegratorForBodyPoint(RigidBody& body, Vec3 point)
{
	Vec3 velocityOfPoint = body.LinearVelocity + cross(body.AngularVelocity, point);

	std::cout << "Linear Velocity of point " << point.x << " " << point.y << " " << point.z << ": "
		<< velocityOfPoint.x << " " << velocityOfPoint.y << " " << velocityOfPoint.z << std::endl;
}

// Calculates orientation as a Quat and sets the value
void RigidBodySystemSimulator::calculateQuat(RigidBody& body, Vec3 axis, float degree)
{
	float radian = degree * 3.14 / 180;
	Quat orientation = Quat(axis, radian);
	body.Orientation = orientation;
}

// Calculates and sets Rotation Matrix given Orientation Q (quad)
void RigidBodySystemSimulator::RotMatrixCalculator(RigidBody& body, Quat Q)
{
	//Mat4d rotMat = Q.getRotMat();


	float q0 = Q.x;
	float q1 = Q.y;
	float q2 = Q.z;
	float q3 = Q.w;

	// First row of the rotation matrix
	float r00 = 2 * (q0 * q0 + q1 * q1) - 1;
	float r01 = 2 * (q1 * q2 - q0 * q3);
	float r02 = 2 * (q1 * q3 + q0 * q2);

	// Second row of the rotation matrix
	float r10 = 2 * (q1 * q2 + q0 * q3);
	float r11 = 2 * (q0 * q0 + q2 * q2) - 1;
	float r12 = 2 * (q2 * q3 - q0 * q1);

	// Third row of the rotation matrix
	float r20 = 2 * (q1 * q3 - q0 * q2);
	float r21 = 2 * (q2 * q3 + q0 * q1);
	float r22 = 2 * (q0 * q0 + q3 * q3) - 1;

	body.RotationMatrix = { {r00,r01,r02}, {r10,r11,r12}, {r20,r21,r22} };
}

void RigidBodySystemSimulator::setupBodies() {
	reset();

	matrix4x4<double> rotMat = matrix4x4<double>();

	addRigidBody(Vec3(-0.2, 0.4, 0.2), Vec3(0.2, 0.12, 0.1), 2);
	rotMat.initRotationZ(90);
	setOrientationOf(0, Quat(rotMat));

	addRigidBody(Vec3(0.7, 0.65, 0), Vec3(0.3, 0.3, 0.3), 2);
	//setVelocityOf(1, Vec3(-2.0f, 0.4f, 0.2f));
	rotMat.initRotationXYZ(0, 45, 45);
	setOrientationOf(1, Quat(rotMat));
}

/**
 * This function is called once when the UI is initialized. It is used to add variables to the tweak
 * bar
 *
 * @param DUC A pointer to the DrawingUtilitiesClass object.
 */
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	//TwAddVarRW(DUC->g_pTweakBar, "Bounciness with spring", TW_TYPE_FLOAT, &m_fBouncinessWithSpringSystem, "min=0.0 step=0.01");
}

/**
 * It resets the current scene
 */
void RigidBodySystemSimulator::reset()
{
	m_externalForce = Vec3(0, 0, 0);
	rigidArr.clear();
}

/**
 * It draws the points and the lines between them
 */
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {

	for (int i = 0; i < rigidArr.size(); i++) {
		RigidBody body = rigidArr.at(i);
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, body.color);
		DUC->drawRigidBody(body.m_objToWorld);
	}
}

/**
 * This function is called when the user changes the test case (the demo)
 *
 * @param testCase The test case number that you want to run.
 */
void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	setupBodies();
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
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
		float inputScale = 0.05f;
		inputWorld = inputWorld * inputScale;
		for (int i = 0; i < rigidArr.size(); i++) {
			applyForceOnBody(i, rigidArr[i].Position, inputWorld);
		}
	}
}

/**
 * This function is used to simulate the time step of the system
 *
 * @param timeStep The time step to simulate.
 */
void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	//update values
	for (int i = 0; i < rigidArr.size(); i++) {
		//apply gravity
		RigidBody& body = rigidArr[i];
		applyForceOnBody(i, rigidArr[i].Position, Vec3(0, -10 * rigidArr[i].Mass, 0));

		torqueIntegrator(body);
		positionIntegrator(body, timeStep);
		linearVelocityIntegrator(body, timeStep);
		angularMomentumIntegrator(body, timeStep);


		inertiaTensorIntegrator(body);

		angularVelocityIntegrator(body);
		orientationIntegrator(body, timeStep);

		body.appliedForces.clear();
		body.Torque = Vec3(0, 0, 0);
		
		body.updateWorldMatrix();
	}

	for (int i = 0; i < panelArr->size(); i++) {
		RigidBody& body = panelArr->at(i);
		torqueIntegrator(body);
		positionIntegrator(body, timeStep);
		linearVelocityIntegrator(body, timeStep);
		angularMomentumIntegrator(body, timeStep);


		inertiaTensorIntegrator(body);

		angularVelocityIntegrator(body);
		orientationIntegrator(body, timeStep);

		body.appliedForces.clear();
		body.Torque = Vec3(0, 0, 0);

		body.updateWorldMatrix();
	}

	handleCollisions();
	handleCollisionsWithSpringSystem();
}

void RigidBodySystemSimulator::handleCollisions() {

	for (int i = 0; i < rigidArr.size(); i++) {
		RigidBody a = rigidArr[i];

		for (int j = i + 1; j < rigidArr.size(); j++) {
			RigidBody b = rigidArr[j];

			CollisionInfo result = checkCollisionSAT(a.m_objToWorld, b.m_objToWorld);

			if (result.isValid) {
				Vec3 x_a = result.collisionPointWorld - a.Position;
				Vec3 v_a = a.LinearVelocity + cross(a.AngularVelocity, x_a);
				Vec3 x_b = result.collisionPointWorld - b.Position;
				Vec3 v_b = b.LinearVelocity + cross(b.AngularVelocity, x_b);

				float relVelonNormal = dot(v_a - v_b, result.normalWorld);
				if (relVelonNormal > 0.0f) continue; // leaving each other, collide before

				Vec3 v_rel = v_a - v_b;
				float jNumerator = -(1 + m_fBounciness) * relVelonNormal;
				Vec3 jDenominatorRotationalImpactA = cross(a.invertedInertiaTensor.transformVector(cross(x_a, result.normalWorld)), x_a);
				Vec3 jDenominatorRotationalImpactB = cross(b.invertedInertiaTensor.transformVector(cross(x_b, result.normalWorld)), x_b);
				float jDenominator = (1 / a.Mass) + (1 / b.Mass) + dot(jDenominatorRotationalImpactA + jDenominatorRotationalImpactB, result.normalWorld);

				float J = jNumerator / jDenominator;

				rigidArr[i].LinearVelocity += (J * result.normalWorld) / a.Mass;
				rigidArr[j].LinearVelocity -= (J * result.normalWorld) / b.Mass;

				rigidArr[i].AngularMomentum += cross(x_a, J * result.normalWorld);
				rigidArr[j].AngularMomentum -= cross(x_b, J * result.normalWorld);
			}
		}
	}
}

void RigidBodySystemSimulator::handleCollisionsWithSpringSystem() {

	for (int i = 0; i < rigidArr.size(); i++) {
		RigidBody a = rigidArr[i];

		for (int j = 0; j < panelArr->size(); j++) {
			RigidBody b = panelArr->at(j);

			CollisionInfo result = checkCollisionSAT(a.m_objToWorld, b.m_objToWorld);

			if (result.isValid) {
				Vec3 x_a = result.collisionPointWorld - a.Position;
				Vec3 v_a = a.LinearVelocity + cross(a.AngularVelocity, x_a);
				Vec3 x_b = result.collisionPointWorld - b.Position;
				Vec3 v_b = b.LinearVelocity + cross(b.AngularVelocity, x_b);

				float relVelonNormal = dot(v_a - v_b, result.normalWorld);
				if (relVelonNormal > 0.0f) continue; // leaving each other, collide before

				Vec3 v_rel = v_a - v_b;
				float jNumerator = -(m_fBouncinessWithSpringSystem) * relVelonNormal;
				Vec3 jDenominatorRotationalImpactA = cross(a.invertedInertiaTensor.transformVector(cross(x_a, result.normalWorld)), x_a);
				Vec3 jDenominatorRotationalImpactB = cross(b.invertedInertiaTensor.transformVector(cross(x_b, result.normalWorld)), x_b);
				float jDenominator = (1 / a.Mass) + (1 / b.Mass) + dot(jDenominatorRotationalImpactA + jDenominatorRotationalImpactB, result.normalWorld);

				float J = jNumerator / jDenominator;

				rigidArr[i].LinearVelocity = a.LinearVelocity + (J * result.normalWorld) / a.Mass;
				//panelArr->at(j).LinearVelocity -= (J * result.normalWorld) / b.Mass;

				rigidArr[i].AngularMomentum += cross(x_a, J * result.normalWorld);
				//panelArr->at(j).AngularMomentum -= cross(x_b, J * result.normalWorld);

				for (int i = 0; i < 4; i++) {
					pointArr->at(panelArr->at(j).panelCornerPoints[i]).Velocity -= (J * result.normalWorld) / (b.Mass * 4.0f);
				}
			}
		}
	}
}