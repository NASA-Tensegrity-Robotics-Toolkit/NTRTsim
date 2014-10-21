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
 * @file MuscleNP.cpp
 * @brief Definition of a massless cable with contact dynamics
 * $Id$
 */

// This object
#include "MuscleNP.h"

// The Bullet Physics library
#include "BulletCollision/CollisionDispatch/btGhostObject.h"

#include <iostream>

MuscleNP::MuscleNP(btPairCachingGhostObject* ghostObject,
 btRigidBody * body1,
 btVector3 pos1,
 btRigidBody * body2,
 btVector3 pos2,
 double coefK,
 double dampingCoefficient) :
Muscle2P (body1, pos1, body2, pos2, coefK, dampingCoefficient),
m_ghostObject(ghostObject)
{
	
}
         
MuscleNP::~MuscleNP()
{
	
}

btVector3 MuscleNP::calculateAndApplyForce(double dt)
{
	std::cout << m_ghostObject->getOverlappingPairCache()->getNumOverlappingPairs() << std::endl;
	if (m_ghostObject->getOverlappingPairCache()->getNumOverlappingPairs() > 0)
	{
		std::cout << m_ghostObject->getOverlappingObject(0) << std::endl;
	}
	// Update the transform so it moves with its anchors. - How will splitting it up affect this??
	
	Muscle2P::calculateAndApplyForce(dt);
}
