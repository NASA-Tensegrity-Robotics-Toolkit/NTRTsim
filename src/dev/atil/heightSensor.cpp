/*
 * heightSensor.cpp
 *
 *  Created on: Jul 15, 2014
 *      Author: atiliscen
 */

#include "heightSensor.h"


heightSensor::heightSensor(){

}

//Place the marker to the current world position and attach it to the body.
heightSensor::heightSensor(const btRigidBody *body,btVector3 worldPos,int nodeNumber,tgWorld& world)
{
	btworld=&(tgBulletUtil::worldToDynamicsWorld(world));
	attachedBody=body;
	//find relative position
	attachedRelativeOriginalPosition=worldPos;//Use local transform;
	this->nodeNumber = nodeNumber;
}

//This returns current position relative to the rigidbody.
btVector3 heightSensor::getRelativePosition() const
{
	btTransform tr = attachedBody->getWorldTransform();
	btVector3 worldPos = tr * attachedRelativeOriginalPosition;
	return worldPos-this->attachedBody->getCenterOfMassPosition();
}

//This returns current absolute position of the marker.
btVector3 heightSensor::getWorldPosition() const
{
	btTransform tr = attachedBody->getWorldTransform();
	return tr * attachedRelativeOriginalPosition;
}

double heightSensor::getHeight() const
{
	double distance = -1.0;

	btVector3 origin=this->getWorldPosition();
	btVector3 end = origin;
	end.setY(-5000.0f);
	origin.setY(origin.getY()-1);
	btCollisionWorld::AllHitsRayResultCallback rayCallback(origin, end);
	btworld->rayTest(origin, end, rayCallback);
	if(rayCallback.hasHit())
	{
		int idx = 0;
		rayCallback.m_hitPointWorld[idx] = origin - rayCallback.m_hitPointWorld[idx];
		distance=rayCallback.m_hitPointWorld[idx].getY();
	}
	return distance;
}
