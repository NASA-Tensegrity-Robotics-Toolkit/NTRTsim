/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file abstractMarker.cpp
 * @brief Markers for specific places on a tensegrity
 * @author Atil Iscen
 * @date Jul 15, 2014
 * $Id$
 */
 
#include "abstractMarker.h"

abstractMarker::abstractMarker(){
    // Supress compiler warning for bullet's unused variable
    (void) btInfinityMask;
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

