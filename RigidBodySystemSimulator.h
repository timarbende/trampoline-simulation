#pragma once
#include "Simulator.h"
#include "RigidBody.h"
#include "Panel.h"


class RigidBodySystemSimulator :public Simulator {
public:

	// Functions
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	const char* getTestCasesStr() {
		return "Project";
	}
	void onClick(int x, int y) {}
	void onMouse(int x, int y) {}

	// ExtraFunctions
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	void handleCollisions();
	void handleCollisionsWithSpringSystem();

	std::vector<RigidBody> rigidArr;
	std::vector<Panel>* panelArr;
	std::vector<MassPoint>* pointArr;

private:
	Vec3 m_externalForce = Vec3(0, 0, 0);
	float m_fBounciness = 1.0f;
	float m_fBouncinessWithSpringSystem = 0.015f;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	//functions
	void setupBodies();
	void orientationIntegrator(RigidBody& body, float timestep);
	void positionIntegrator(RigidBody& body, float timestep);
	void angularMomentumIntegrator(RigidBody& body, float timestep);
	void inertiaTensorIntegrator(RigidBody& body);
	void angularVelocityIntegrator(RigidBody& body);
	void linearVelocityIntegrator(RigidBody& body, float timestep);
	void linearVelocityIntegratorForBodyPoint(RigidBody& body, Vec3 point);
	void calculateQuat(RigidBody& body, Vec3 axis, float degree);
	void RotMatrixCalculator(RigidBody& body, Quat Q);
	void torqueIntegrator(RigidBody& body);
	std::vector<std::vector<float>> inverseMatrix3x3(std::vector<std::vector<float>> m);
	std::vector<std::vector<float>> transposeMatrix3x3(std::vector<std::vector<float>> m);
};
