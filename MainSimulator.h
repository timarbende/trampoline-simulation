#pragma once
#include "Simulator.h"
#include "RigidBodySystemSimulator.h"
#include "MassSpringSystemSimulator.h"

class MainSimulator : public Simulator
{
public:
	MassSpringSystemSimulator massSpringSystemSimulator;
	RigidBodySystemSimulator rigidBodySystemSimulator;

	MainSimulator() {
		rigidBodySystemSimulator.pointArr = &massSpringSystemSimulator.pointArr;
		rigidBodySystemSimulator.panelArr = &massSpringSystemSimulator.panelArr;
	}

	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	const char* getTestCasesStr() {
		return "Project";
	}
	void onClick(int x, int y);
	void onMouse(int x, int y);
};

