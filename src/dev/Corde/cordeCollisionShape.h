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

#ifndef CORDE_COLLISION_SHAPE
#define CORDE_COLLISION_SHAPE

/**
 * @file cordeCollisionShape.h
 * @brief Collision shape for the Corde Collision object
 * Largely based on bullet's btSoftBodyCollisionShape in btSoftBodyInternals.h
 * @author Brian Mirletz
 * $Id$
 */

// Bullet Physics
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "LinearMath/btVector3.h"

// Can this be forward declared??
#include "cordeCollisionObject.h"

class cordeCollisionObject;

class cordeCollisionShape : public btCollisionShape
{
public:

	cordeCollisionShape(cordeCollisionObject* objectShape);
	
	virtual ~cordeCollisionShape() { }
	
	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
	{
		///@todo define this once we know how to calculate bounds from cordeCollisionObject
	}
	
	virtual void	setLocalScaling(const btVector3& scaling) { }/// @todo 
	virtual const btVector3& getLocalScaling() { }/// @todo 
	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia) const { }/// @todo 
	virtual const char*	getName() const { return "cordeCollisionShape"; }/// @todo 
	virtual void	setMargin(btScalar margin) { } /// @todo 
	virtual btScalar	getMargin() const { return 0.0; } /// @todo 
	
private:
	
	cordeCollisionObject* p_objectShape;
};

#endif // CORDE_COLLISION_SHAPE
