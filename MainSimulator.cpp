#include "MainSimulator.h"
#include "collisionDetect.h"
#include "Panel.h"

void MainSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	massSpringSystemSimulator.initUI(DUC);
	rigidBodySystemSimulator.initUI(DUC);
}

void MainSimulator::reset()
{
	massSpringSystemSimulator.reset();
	rigidBodySystemSimulator.reset();
}

void MainSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	massSpringSystemSimulator.drawFrame(pd3dImmediateContext);
	rigidBodySystemSimulator.drawFrame(pd3dImmediateContext);
}

void MainSimulator::notifyCaseChanged(int testCase)
{
	massSpringSystemSimulator.notifyCaseChanged(testCase);
	rigidBodySystemSimulator.notifyCaseChanged(testCase);
}

void MainSimulator::externalForcesCalculations(float timeElapsed)
{
	massSpringSystemSimulator.externalForcesCalculations(timeElapsed);
	rigidBodySystemSimulator.externalForcesCalculations(timeElapsed);
}

void MainSimulator::simulateTimestep(float timeStep)
{
	massSpringSystemSimulator.simulateTimestep(timeStep);
	rigidBodySystemSimulator.simulateTimestep(timeStep);
}

void MainSimulator::onClick(int x, int y)
{
	//massSpringSystemSimulator.onClick(x, y);
	rigidBodySystemSimulator.onClick(x, y);
}

void MainSimulator::onMouse(int x, int y)
{
	//massSpringSystemSimulator.onMouse(x, y);
	//rigidBodySystemSimulator.onMouse(x, y);
}
