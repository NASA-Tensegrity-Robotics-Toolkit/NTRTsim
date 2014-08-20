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

This class is a modified version of btSoftRigidDynamicsWorld
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

#ifndef CORDE_RIGID_DYNAMICS_WORLD_H
#define CORDE_RIGID_DYNAMICS_WORLD_H

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

class cordeCollisionObject;

typedef	btAlignedObjectArray<cordeCollisionObject*> btCordeObjectArray;

class cordeSolver;

class cordeDynamicsWorld : public btDiscreteDynamicsWorld
{

	btCordeObjectArray	m_cordeObjects;
	///Solver classes that encapsulate multiple soft bodies for solving
	cordeSolver *m_softBodySolver;
	bool			m_ownsSolver;

protected:

	virtual void	predictUnconstraintMotion(btScalar timeStep);

	virtual void	internalSingleStepSimulation( btScalar timeStep);

	void	solveSoftBodiesConstraints( btScalar timeStep );

	void	serializeSoftBodies(btSerializer* serializer);

public:

	cordeDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration, cordeSolver *softBodySolver = 0 );

	virtual ~cordeDynamicsWorld();

	virtual void	debugDrawWorld();

	void	addSoftBody(cordeCollisionObject* body,short int collisionFilterGroup=btBroadphaseProxy::DefaultFilter,short int collisionFilterMask=btBroadphaseProxy::AllFilter);

	void	removeSoftBody(cordeCollisionObject* body);

	///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btDiscreteDynamicsWorld::removeCollisionObject
	virtual void	removeCollisionObject(btCollisionObject* collisionObject);

	/**
	 * Changing this to something unique would require adding to the 
	 * enum in btDynamicsWorld.h
	 */
	virtual btDynamicsWorldType	getWorldType() const
	{
		return	BT_SOFT_RIGID_DYNAMICS_WORLD;
	}

	btCordeObjectArray& getSoftBodyArray()
	{
		return m_cordeObjects;
	}

	const btCordeObjectArray& getSoftBodyArray() const
	{
		return m_cordeObjects;
	}

#if (0)
	virtual void rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback) const; 

	/// rayTestSingle performs a raycast call and calls the resultCallback. It is used internally by rayTest.
	/// In a future implementation, we consider moving the ray test as a virtual method in btCollisionShape.
	/// This allows more customization.
	static void	rayTestSingle(const btTransform& rayFromTrans,const btTransform& rayToTrans,
					  btCollisionObject* collisionObject,
					  const btCollisionShape* collisionShape,
					  const btTransform& colObjWorldTransform,
					  RayResultCallback& resultCallback);
#endif // Temp disable raytest and serialize
	virtual	void	serialize(btSerializer* serializer);

};

#endif //CORDE_RIGID_DYNAMICS_WORLD_H
