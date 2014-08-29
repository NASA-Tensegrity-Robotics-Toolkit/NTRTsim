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

This class is a modified version of btSoftBodyCollisionShape
* from Bullet 2.82. A copy of the z-lib license from the Bullet Physics
* Library is provided below:

Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
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

class cordeCollisionObject;

class cordeCollisionShape : public btCollisionShape
{
public:

	cordeCollisionShape(cordeCollisionObject* objectShape);
	
	virtual ~cordeCollisionShape()
	{ 
		p_objectShape = NULL;
	}
	
	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;
	
	virtual void	setLocalScaling(const btVector3& scaling); ///@todo - 
	
	virtual const btVector3& getLocalScaling() const;
	
	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia) const { }/// @todo 
	
	virtual const char*	getName() const 
	{ 
		return "cordeCollisionShape"; 
	} 
	
	virtual void	setMargin(btScalar margin) 
	{
		/// @todo check input, must be >= 0
		m_collisionMargin = margin;
	} 
	
	virtual btScalar	getMargin() const 
	{
		return m_collisionMargin;
	}
	
private:
	
	/** Backpointer, we don't own this */
	cordeCollisionObject* p_objectShape;
	
	btScalar m_collisionMargin;
	
	btVector3 localScaling;
};

#endif // CORDE_COLLISION_SHAPE
