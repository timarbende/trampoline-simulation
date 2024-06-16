#pragma once

#include "Simulator.h"

struct Force {
public:
	Vec3 location;
	Vec3 direction;
};

class RigidBody {
public:
	Vec3 Position;
	Vec3 Size;
	Vec3 LinearVelocity;
	Vec3 AngularVelocity;
	std::vector<Force> appliedForces;
	Vec3 Torque;
	Vec3 AngularMomentum;
	int Mass = 1;
	matrix4x4<double> invertedInitialInertiaTensor;
	matrix4x4<double> invertedInertiaTensor;
	Quat Orientation;
	std::vector<std::vector<float>> RotationMatrix;
	std::vector<Vec3> CornerPoints;

	Vec3 color = Vec3(0.9, 0.97, 1);

	GamePhysics::Mat4 m_objToWorld;
	GamePhysics::Mat4 m_worldToObj;
	GamePhysics::Mat4 m_scaledObjToWorld;
	GamePhysics::Mat4 m_worldToScaledObj;


	void calculateInitialInertiaTensor() {
		invertedInitialInertiaTensor.initId();

		matrix4x4<double> covariance = matrix4x4<double>();
		covariance.initId();

		calcCornerPoints();
		for (int i = 0; i < CornerPoints.size(); i++)
		{
			Vec3 cornerPoint = CornerPoints[i];
			covariance.value[0][0] += Mass * cornerPoint.x * cornerPoint.x;
			covariance.value[0][1] += Mass * cornerPoint.x * cornerPoint.y;
			covariance.value[0][2] += Mass * cornerPoint.x * cornerPoint.z;
			covariance.value[1][0] += Mass * cornerPoint.y * cornerPoint.x;
			covariance.value[1][1] += Mass * cornerPoint.y * cornerPoint.y;
			covariance.value[1][2] += Mass * cornerPoint.y * cornerPoint.z;
			covariance.value[2][0] += Mass * cornerPoint.z * cornerPoint.x;
			covariance.value[2][1] += Mass * cornerPoint.z * cornerPoint.y;
			covariance.value[2][2] += Mass * cornerPoint.z * cornerPoint.z;
		}

		double trace = covariance.value[0][0] + covariance.value[1][1] + covariance.value[2][2];
		matrix4x4<double> identity = matrix4x4<double>();
		identity.initId();
		invertedInitialInertiaTensor = identity * trace - covariance;

		invertedInitialInertiaTensor = invertedInitialInertiaTensor.inverse();
	}

	RigidBody(Vec3 pos, Vec3 size, int m) {
		Position = pos;
		Size = size;
		Mass = m;
		invertedInitialInertiaTensor = matrix4x4<double>();
		invertedInertiaTensor = matrix4x4<double>();
		calculateInitialInertiaTensor();
		updateWorldMatrix();
	}

	void updateWorldMatrix();

private:
	void calcCornerPoints()
	{
		CornerPoints.clear();
		Vec3 center = Position;
		Vec3 size = Size / 2.0f;
		Vec3 p1 = Position + Orientation.getRotMat() * Vec3(size.x, size.y, size.z);
		Vec3 p2 = Position + Orientation.getRotMat() * Vec3(-size.x, size.y, size.z);
		Vec3 p3 = Position + Orientation.getRotMat() * Vec3(-size.x, size.y, -size.z);
		Vec3 p4 = Position + Orientation.getRotMat() * Vec3(size.x, size.y, -size.z);
		Vec3 p5 = Position + Orientation.getRotMat() * Vec3(size.x, -size.y, size.z);
		Vec3 p6 = Position + Orientation.getRotMat() * Vec3(-size.x, -size.y, size.z);
		Vec3 p7 = Position + Orientation.getRotMat() * Vec3(-size.x, -size.y, -size.z);
		Vec3 p8 = Position + Orientation.getRotMat() * Vec3(size.x, -size.y, -size.z);

		CornerPoints.push_back(p1);
		CornerPoints.push_back(p2);
		CornerPoints.push_back(p3);
		CornerPoints.push_back(p4);
		CornerPoints.push_back(p5);
		CornerPoints.push_back(p6);
		CornerPoints.push_back(p7);
		CornerPoints.push_back(p8);
	}
};

