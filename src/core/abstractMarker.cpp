/*
 * abstractMarker.cpp
 *
 *  Created on: Jul 15, 2014
 *      Author: atiliscen
 */

#include "abstractMarker.h"

abstractMarker::abstractMarker(){

}

//Place the marker to the current world position and attach it to the body.
abstractMarker::abstractMarker(const btRigidBody *body,btVector3 worldPos,btVector3 color,int nodeNumber)
{
	attachedBody=body;
	//find relative position
	attachedRelativeOriginalPosition=worldPos;//Use local transform;
	this->color = color;
	this->nodeNumber = nodeNumber;
}

//This returns current position relative to the rigidbody.
btVector3 abstractMarker::getRelativePosition() const
{
	btTransform tr = attachedBody->getWorldTransform();
	btVector3 worldPos = tr * attachedRelativeOriginalPosition;
	return worldPos-this->attachedBody->getCenterOfMassPosition();
}

//This returns current absolute position of the marker.
btVector3 abstractMarker::getWorldPosition() const
{
	btTransform tr = attachedBody->getWorldTransform();
	return tr * attachedRelativeOriginalPosition;
}

