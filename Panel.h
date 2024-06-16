#pragma once
#include "RigidBody.h"
#include "MassPoint.h"

class Panel : public RigidBody{
public:
	int panelCornerPoints[4];

	Panel(int p1, int p2, int p3, int p4, RigidBody rigidBody) : RigidBody(rigidBody) {
		panelCornerPoints[0] = p1;
		panelCornerPoints[1] = p2;
		panelCornerPoints[2] = p3;
		panelCornerPoints[3] = p4;
	}
};

