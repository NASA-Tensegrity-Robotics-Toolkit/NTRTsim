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

This class is a modified version of btSoftRigidCollisionAlgorithm
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

#ifndef CORDE_RIGID_COLLISION_ALGORITHM_H
#define CORDE_RIGID_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
class btPersistentManifold;
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

#include "LinearMath/btVector3.h"
class cordeCollisionObject;

/// cordeRigidCollisionAlgorithm  provides collision detection between cordeCollisionObject and btRigidBody
class cordeRigidCollisionAlgorithm : public btCollisionAlgorithm
{
	//	bool	m_ownManifold;
	//	btPersistentManifold*	m_manifoldPtr;

	cordeCollisionObject*	m_cordeObject;
	btCollisionObject*		m_rigidCollisionObject;

	///for rigid versus soft (instead of soft versus rigid), we use this swapped boolean
	bool	m_isSwapped;

public:

	cordeRigidCollisionAlgorithm(btPersistentManifold* mf,const btCollisionAlgorithmConstructionInfo& ci,const btCollisionObjectWrapper* col0,const btCollisionObjectWrapper* col1Wrap, bool isSwapped);

	virtual ~cordeRigidCollisionAlgorithm();

	virtual void processCollision (const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btManifoldArray&	manifoldArray)
	{
		//we don't add any manifolds
	}


	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cordeRigidCollisionAlgorithm));
			if (!m_swapped)
			{
				return new(mem) cordeRigidCollisionAlgorithm(0,ci,body0Wrap,body1Wrap,false);
			} else
			{
				return new(mem) cordeRigidCollisionAlgorithm(0,ci,body0Wrap,body1Wrap,true);
			}
		}
	};

};

#endif //CORDE_RIGID_COLLISION_ALGORITHM_H


