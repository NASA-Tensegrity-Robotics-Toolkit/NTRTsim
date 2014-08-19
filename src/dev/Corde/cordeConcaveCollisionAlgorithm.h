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

This class is a modified version of btSoftBodyConcaveCollisionAlgorithm
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

#ifndef CORDE_CONCAVE_COLLISION_ALGORITHM_H
#define CORDE_CONCAVE_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionShapes/btTriangleCallback.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class btDispatcher;
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
class btSoftBody;
class btCollisionShape;

#include "LinearMath/btHashMap.h"

#include "BulletCollision/BroadphaseCollision/btQuantizedBvh.h" //for definition of MAX_NUM_PARTS_IN_BITS

struct btTriIndex
{
	int m_PartIdTriangleIndex;
	class btCollisionShape*	m_childShape;

	btTriIndex(int partId,int triangleIndex,btCollisionShape* shape)
	{
		m_PartIdTriangleIndex = (partId<<(31-MAX_NUM_PARTS_IN_BITS)) | triangleIndex;
		m_childShape = shape;
	}

	int	getTriangleIndex() const
	{
		// Get only the lower bits where the triangle index is stored
		unsigned int x = 0;
		unsigned int y = (~(x&0))<<(31-MAX_NUM_PARTS_IN_BITS);
		return (m_PartIdTriangleIndex&~(y));
	}
	int	getPartId() const
	{
		// Get only the highest bits where the part index is stored
		return (m_PartIdTriangleIndex>>(31-MAX_NUM_PARTS_IN_BITS));
	}
	int	getUid() const
	{
		return m_PartIdTriangleIndex;
	}
};


///For each triangle in the concave mesh that overlaps with the AABB of a soft body (m_softBody), processTriangle is called.
class cordeTriangleCallback : public btTriangleCallback
{
	btSoftBody* m_softBody;
	const btCollisionObject* m_triBody;

	btVector3	m_aabbMin;
	btVector3	m_aabbMax ;

	btManifoldResult* m_resultOut;

	btDispatcher*	m_dispatcher;
	const btDispatcherInfo* m_dispatchInfoPtr;
	btScalar m_collisionMarginTriangle;

	btHashMap<btHashKey<btTriIndex>,btTriIndex> m_shapeCache;

public:
	int	m_triangleCount;

	//	btPersistentManifold*	m_manifoldPtr;

	cordeTriangleCallback(btDispatcher* dispatcher,const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,bool isSwapped);

	void	setTimeStepAndCounters(btScalar collisionMarginTriangle,const btCollisionObjectWrapper* triObjWrap,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual ~cordeTriangleCallback();

	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex);

	void clearCache();

	SIMD_FORCE_INLINE const btVector3& getAabbMin() const
	{
		return m_aabbMin;
	}
	SIMD_FORCE_INLINE const btVector3& getAabbMax() const
	{
		return m_aabbMax;
	}

};




/// btSoftBodyConcaveCollisionAlgorithm  supports collision between soft body shapes and (concave) trianges meshes.
class cordeConcaveCollisionAlgorithm  : public btCollisionAlgorithm
{

	bool	m_isSwapped;

	cordeTriangleCallback m_cordeTriangleCallback;

public:

	cordeConcaveCollisionAlgorithm( const btCollisionAlgorithmConstructionInfo& ci,const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,bool isSwapped);

	virtual ~cordeConcaveCollisionAlgorithm();

	virtual void processCollision (const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	btScalar	calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btManifoldArray&	manifoldArray)
	{
		//we don't add any manifolds
	}

	void	clearCache();

	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cordeConcaveCollisionAlgorithm));
			return new(mem) cordeConcaveCollisionAlgorithm(ci,body0Wrap,body1Wrap,false);
		}
	};

	struct SwappedCreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cordeConcaveCollisionAlgorithm));
			return new(mem) cordeConcaveCollisionAlgorithm(ci,body0Wrap,body1Wrap,true);
		}
	};

};

#endif //BT_SOFT_BODY_CONCAVE_COLLISION_ALGORITHM_H
