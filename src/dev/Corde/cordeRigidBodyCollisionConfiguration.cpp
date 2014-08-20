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

This class is a modified version of btSoftBodyRigidBodyCollisionConfiguration
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
#include "cordeRigidBodyCollisionConfiguration.h"
#include "cordeRigidCollisionAlgorithm.h"
//#include "cordeConcaveCollisionAlgorithm.h" (not yet)
#include "cordeSelfCollisionAlgorithm.h"

#include "LinearMath/btPoolAllocator.h"

//#define ENABLE_SOFTBODY_CONCAVE_COLLISIONS

cordeRigidBodyCollisionConfiguration::cordeRigidBodyCollisionConfiguration(const btDefaultCollisionConstructionInfo& constructionInfo)
:btDefaultCollisionConfiguration(constructionInfo)
{
	void* mem;

	mem = btAlignedAlloc(sizeof(cordeSelfCollisionAlgorithm::CreateFunc),16);
	m_cordeSelfCreateFunc = new(mem) cordeSelfCollisionAlgorithm::CreateFunc;

	mem = btAlignedAlloc(sizeof(cordeRigidCollisionAlgorithm::CreateFunc),16);
	m_cordeRigidConvexCreateFunc = new(mem) cordeRigidCollisionAlgorithm::CreateFunc;

	mem = btAlignedAlloc(sizeof(cordeRigidCollisionAlgorithm::CreateFunc),16);
	m_swappedCordeRigidConvexCreateFunc = new(mem) cordeRigidCollisionAlgorithm::CreateFunc;
	m_swappedCordeRigidConvexCreateFunc->m_swapped=true;

#ifdef ENABLE_SOFTBODY_CONCAVE_COLLISIONS
	mem = btAlignedAlloc(sizeof(cordeConcaveCollisionAlgorithm::CreateFunc),16);
	m_cordeRigidConcaveCreateFunc = new(mem) cordeConcaveCollisionAlgorithm::CreateFunc;

	mem = btAlignedAlloc(sizeof(cordeConcaveCollisionAlgorithm::CreateFunc),16);
	m_swappedCordeRigidConcaveCreateFunc = new(mem) cordeConcaveCollisionAlgorithm::SwappedCreateFunc;
	m_swappedCordeRigidConcaveCreateFunc->m_swapped=true;
#endif

	//replace pool by a new one, with potential larger size

	if (m_ownsCollisionAlgorithmPool && m_collisionAlgorithmPool)
	{
		int curElemSize = m_collisionAlgorithmPool->getElementSize();
		///calculate maximum element size, big enough to fit any collision algorithm in the memory pool


		int maxSize0 = sizeof(cordeSelfCollisionAlgorithm);
		int maxSize1 = sizeof(cordeRigidCollisionAlgorithm);
#ifdef ENABLE_SOFTBODY_CONCAVE_COLLISIONS		
		int maxSize2 = sizeof(cordeConcaveCollisionAlgorithm);
#endif
		int	collisionAlgorithmMaxElementSize = btMax(maxSize0,maxSize1);
#ifdef ENABLE_SOFTBODY_CONCAVE_COLLISIONS		
		collisionAlgorithmMaxElementSize = btMax(collisionAlgorithmMaxElementSize,maxSize2);
#endif		
		if (collisionAlgorithmMaxElementSize > curElemSize)
		{
			m_collisionAlgorithmPool->~btPoolAllocator();
			btAlignedFree(m_collisionAlgorithmPool);
			void* mem = btAlignedAlloc(sizeof(btPoolAllocator),16);
			m_collisionAlgorithmPool = new(mem) btPoolAllocator(collisionAlgorithmMaxElementSize,constructionInfo.m_defaultMaxCollisionAlgorithmPoolSize);
		}
	}

}

cordeRigidBodyCollisionConfiguration::~cordeRigidBodyCollisionConfiguration()
{
	m_cordeSelfCreateFunc->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(	m_cordeSelfCreateFunc);

	m_cordeRigidConvexCreateFunc->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(	m_cordeRigidConvexCreateFunc);

	m_swappedCordeRigidConvexCreateFunc->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(	m_swappedCordeRigidConvexCreateFunc);

#ifdef ENABLE_SOFTBODY_CONCAVE_COLLISIONS
	m_cordeRigidConcaveCreateFunc->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(	m_cordeRigidConcaveCreateFunc);

	m_swappedCordeRigidConcaveCreateFunc->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(	m_swappedCordeRigidConcaveCreateFunc);
#endif
}

///creation of soft-soft and soft-rigid, and otherwise fallback to base class implementation
btCollisionAlgorithmCreateFunc* cordeRigidBodyCollisionConfiguration::getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1)
{

	///try to handle the softbody interactions first

	if ((proxyType0 == SOFTBODY_SHAPE_PROXYTYPE  ) && (proxyType1==SOFTBODY_SHAPE_PROXYTYPE))
	{
		return	m_cordeSelfCreateFunc;
	}

	///softbody versus convex
	if (proxyType0 == SOFTBODY_SHAPE_PROXYTYPE  && btBroadphaseProxy::isConvex(proxyType1))
	{
		return	m_cordeRigidConvexCreateFunc;
	}

	///convex versus soft body
	if (btBroadphaseProxy::isConvex(proxyType0) && proxyType1 == SOFTBODY_SHAPE_PROXYTYPE )
	{
		return	m_swappedCordeRigidConvexCreateFunc;
	}

#ifdef ENABLE_SOFTBODY_CONCAVE_COLLISIONS
	///softbody versus convex
	if (proxyType0 == SOFTBODY_SHAPE_PROXYTYPE  && btBroadphaseProxy::isConcave(proxyType1))
	{
		return	m_cordeRigidConcaveCreateFunc;
	}

	///convex versus soft body
	if (btBroadphaseProxy::isConcave(proxyType0) && proxyType1 == SOFTBODY_SHAPE_PROXYTYPE )
	{
		return	m_swappedCordeRigidConcaveCreateFunc;
	}
#endif

	///fallback to the regular rigid collision shape
	return btDefaultCollisionConfiguration::getCollisionAlgorithmCreateFunc(proxyType0,proxyType1);
}
