#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "Panel.h"
#include "MassPoint.h"
#include "Spring.h"

class MassSpringSystemSimulator :public Simulator {
public:

	// UI Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	void applyExternalForce(Vec3 force);
	void midPointMethodIntegration(float timeStep);
	void BoundingBoxCheck();

	std::vector<Panel> panelArr;
	std::vector<MassPoint> pointArr;

private:
	// Data Attributes
	int pointIndexMap[11][11];
	std::vector<Spring> springArr;
	float m_fMass = 0.01f;
	float m_fMassPanels = 2.0f;
	float m_fStiffness = 300.0f;
	float m_fDamping = 0.8f;
	int   m_iNumPoints;
	float m_fPointSize = 0.01f;
	int   m_iNumSprings;
	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	// functions
	void setupTrambouline();
	void ComputeForces();
	void updatePanelArray();
};
#endif