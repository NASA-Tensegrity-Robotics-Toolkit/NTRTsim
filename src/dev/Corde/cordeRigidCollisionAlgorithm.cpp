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

// This library
#include "cordeRigidCollisionAlgorithm.h"
#include "cordeCollisionObject.h"
//#include "BulletSoftBody/btSoftBodySolvers.h" - Update solver, abstract or here?

// The Bullet Physics Library
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"


#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

///TODO: include all the shapes that the softbody can collide with
///alternatively, implement special case collision algorithms (just like for rigid collision shapes)

cordeRigidCollisionAlgorithm::cordeRigidCollisionAlgorithm(btPersistentManifold* /*mf*/,const btCollisionAlgorithmConstructionInfo& ci,const btCollisionObjectWrapper* ,const btCollisionObjectWrapper* , bool isSwapped)
: btCollisionAlgorithm(ci),
m_isSwapped(isSwapped)
{
}


cordeRigidCollisionAlgorithm::~cordeRigidCollisionAlgorithm()
{

}


#include <stdio.h>

void cordeRigidCollisionAlgorithm::processCollision (const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	// Avoid compiler warnings
	(void)dispatchInfo;
	(void)resultOut;
	//printf("cordeRigidCollisionAlgorithm\n");
	cordeCollisionObject* cordeObject =  m_isSwapped? (cordeCollisionObject*)body1Wrap->getCollisionObject() : (cordeCollisionObject*)body0Wrap->getCollisionObject();
	const btCollisionObjectWrapper* rigidCollisionObjectWrap = m_isSwapped? body0Wrap : body1Wrap;
	
	/// @todo add these functions
	/*
	if (softBody->m_collisionDisabledObjects.findLinearSearch(rigidCollisionObjectWrap->getCollisionObject())==softBody->m_collisionDisabledObjects.size())
	{
		softBody->getSoftBodySolver()->processCollision(softBody, rigidCollisionObjectWrap);
	}
	*/


}

btScalar cordeRigidCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* col0,btCollisionObject* col1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return btScalar(1.);
}



