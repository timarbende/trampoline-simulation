#include "RigidBody.h"

void RigidBody::updateWorldMatrix() {
	GamePhysics::Mat4 matrix;
	matrix.initTranslation(Position.x, Position.y, Position.z);

	GamePhysics::Mat4 rotation = Orientation.getRotMat();

	m_scaledObjToWorld = rotation * matrix;
	m_worldToScaledObj = m_scaledObjToWorld.inverse();
	matrix.initScaling(Size.x, Size.y, Size.z);
	m_objToWorld = matrix * m_scaledObjToWorld;
	m_worldToObj = m_objToWorld.inverse();
}
