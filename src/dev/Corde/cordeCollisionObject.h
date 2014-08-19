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

#ifndef CORDE_COLLISION_OBJECT
#define CORDE_COLLISION_OBJECT

/**
 * @file CordeCollisionObject.h
 * @brief Interface Between Corde Model and Bullet
 * @author Brian Mirletz
 * $Id$
 */

// Corde Physics
#include "CordeModel.h"

// Bullet Physics
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

// Bullet Linear Algebra
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

// The C++ Standard Library
#include <vector>

class cordeCollisionObject : public CordeModel, public btCollisionObject
{
public:

	cordeCollisionObject(std::vector<btVector3>& centerLine, CordeModel::Config& Config);
	
	virtual ~cordeCollisionObject();

/**
 * @todo implement these members and functions:"
 * if (softBody->m_collisionDisabledObjects.findLinearSearch(rigidCollisionObjectWrap->getCollisionObject())==softBody->m_collisionDisabledObjects.size())
	{
		softBody->getSoftBodySolver()->processCollision(softBody, rigidCollisionObjectWrap);
	}
	* 
	* soft0->getSoftBodySolver()->processCollision(soft0, soft1);
	*
	if (psb->isActive())
		{
			psb->integrateMotion();	
		}
	*
	psb->solveConstraints();
	* 
	softBody->defaultCollisionHandler( otherSoftBody);
	* 
	softBody->defaultCollisionHandler( collisionObjectWrap );
	* 
	psb->predictMotion(timeStep);
	* 
	btSoftBody::upcast(collisionObject) 
*/

};
 
 
#endif // CORDE_MODEL
