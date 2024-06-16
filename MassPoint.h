#pragma once

#include "Simulator.h"

class MassPoint {
public:
	Vec3 Position = Vec3(0, 0, 0);
	Vec3 Velocity = Vec3(0, 0, 0);
	Vec3 Force = Vec3(0, 0, 0);
	Vec3 Acceleration = Vec3(0, 0, 0);
	bool isFixed = false;

	MassPoint() {}

	MassPoint(Vec3 pos, Vec3 vel, Vec3 force, bool fixed) {
		Position = pos;
		Velocity = vel;
		Force = force;
		isFixed = fixed;
	}

	void clearForce() {
		Force = Vec3(0, 0, 0);
		Acceleration = Vec3(0, 0, 0);
	}

};
