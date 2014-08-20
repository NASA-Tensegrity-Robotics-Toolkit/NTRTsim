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

This class is a modified version of btSoftSoftCollisionAlgorithm
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

#include "cordeSelfCollisionAlgorithm.h"
#include "cordeSolvers.h"
#include "cordeCollisionObject.h"

#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

#define USE_PERSISTENT_CONTACTS 1

cordeSelfCollisionAlgorithm::cordeSelfCollisionAlgorithm(btPersistentManifold* /*mf*/,const btCollisionAlgorithmConstructionInfo& ci,const btCollisionObjectWrapper* /*obj0*/,const btCollisionObjectWrapper* /*obj1*/)
: btCollisionAlgorithm(ci)
{
}

cordeSelfCollisionAlgorithm::~cordeSelfCollisionAlgorithm()
{
}

void cordeSelfCollisionAlgorithm::processCollision (const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,const btDispatcherInfo& /*dispatchInfo*/,btManifoldResult* /*resultOut*/)
{
	cordeCollisionObject* soft0 =	(cordeCollisionObject*)body0Wrap->getCollisionObject();
	cordeCollisionObject* soft1 =	(cordeCollisionObject*)body1Wrap->getCollisionObject();
	soft0->getSoftBodySolver()->processCollision(soft0, soft1);
}

btScalar cordeSelfCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* /*body0*/,btCollisionObject* /*body1*/,const btDispatcherInfo& /*dispatchInfo*/,btManifoldResult* /*resultOut*/)
{
	//not yet
	return 1.f;
}
