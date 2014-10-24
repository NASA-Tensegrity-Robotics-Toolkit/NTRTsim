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

// NTRT
#include "tgcreator/tgUtil.h"
#include "core/muscleAnchor.h"
#include "core/tgCast.h"
// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "LinearMath/btDefaultMotionState.h"

// Classes for manual collision detection - will likely remove later
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"

#include <iostream>
#include <algorithm>    // std::sort

MuscleNP::MuscleNP(btPairCachingGhostObject* ghostObject,
btBroadphaseInterface* broadphase,
 btRigidBody * body1,
 btVector3 pos1,
 btRigidBody * body2,
 btVector3 pos2,
 double coefK,
 double dampingCoefficient) :
Muscle2P (body1, pos1, body2, pos2, coefK, dampingCoefficient),
m_ghostObject(ghostObject),
m_overlappingPairCache(broadphase),
m_ac(anchor1, anchor2)
{

}
         
MuscleNP::~MuscleNP()
{
	
}

const btScalar MuscleNP::getActualLength() const
{
    btScalar length = 0;
    
    std::size_t n = m_anchors.size() - 1;
    for (std::size_t i = 0; i < n; i++)
    {
        length += (m_anchors[i]->getWorldPosition() - m_anchors[i+1]->getWorldPosition()).length(); 
    }
    
    return length;
}

btVector3 MuscleNP::calculateAndApplyForce(double dt)
{
	
	updateAnchorList(dt);
	
    // Apply forces
    
    
    
	btVector3 from = anchor1->getWorldPosition();
	btVector3 to = anchor2->getWorldPosition();
	
	btTransform transform = tgUtil::getTransform(from, to);
	
	//std::cout << (to - from).length()/2.0 << std::endl;
	
	m_ghostObject->setWorldTransform(transform);
	
	btScalar radius = 0.01;
	
	btCylinderShape* shape = tgCast::cast<btCollisionShape,  btCylinderShape>(*m_ghostObject->getCollisionShape());
	/* Note that 1) this is listed as "use with care" in Bullet's documentation and
	 * 2) we had to remove the object from DemoApplication's render function in order for it to render properly
	 * changing from a non-contact object will break that behavior.
	 */ 
	shape->setImplicitShapeDimensions(btVector3(radius, (to - from).length()/2.0, radius));
	m_ghostObject->setCollisionShape(shape);
	
	Muscle2P::calculateAndApplyForce(dt);
}

void MuscleNP::updateAnchorList(double dt)
{
	std::vector<const muscleAnchor*>::iterator it = m_anchors.begin();
    muscleAnchor* temp;
	for (it = m_anchors.begin(); it != m_anchors.end(); it++)
	{
		if ((*it)->permanent == false)
        {
            delete *it;
        }
	}
	
    m_anchors.clear();
    
	m_anchors.insert(m_anchors.begin(), anchor1);
	m_anchors.insert(m_anchors.end(), anchor2);
	
	
	
	btManifoldArray	m_manifoldArray;
	btVector3 m_touchingNormal;
	
	//std::cout << m_ghostObject->getOverlappingPairCache()->getNumOverlappingPairs() << std::endl;

	// Only caches the pairs, they don't have a lot of useful information
	btBroadphasePairArray& pairArray = m_ghostObject->getOverlappingPairCache()->getOverlappingPairArray();
	int numPairs = pairArray.size();

	for (int i=0;i<numPairs;i++)
	{
		m_manifoldArray.clear();

		const btBroadphasePair& pair = pairArray[i];
		
		// The real broadphase's pair cache has the useful info
		btBroadphasePair* collisionPair = m_overlappingPairCache->getOverlappingPairCache()->findPair(pair.m_pProxy0,pair.m_pProxy1);

		btCollisionObject* obj0 = static_cast<btCollisionObject*>(collisionPair->m_pProxy0->m_clientObject);
                btCollisionObject* obj1 = static_cast<btCollisionObject*>(collisionPair->m_pProxy1->m_clientObject);

		if (collisionPair->m_algorithm)
			collisionPair->m_algorithm->getAllContactManifolds(m_manifoldArray);
		
		//std::cout  << m_manifoldArray.size() << std::endl;
		
		for (int j=0;j<m_manifoldArray.size();j++)
		{
			btPersistentManifold* manifold = m_manifoldArray[j];
			btScalar directionSign = manifold->getBody0() == m_ghostObject ? btScalar(-1.0) : btScalar(1.0);
			for (int p=0;p<manifold->getNumContacts();p++)
			{
				const btManifoldPoint&pt = manifold->getContactPoint(p);

				btScalar dist = pt.getDistance();
				
				// Ensures the force is pointed outwards - may need to double check
				if (dist < 0.0)
				{
					
					m_touchingNormal = pt.m_normalWorldOnB * directionSign;//??
					
					btRigidBody* rb = NULL;
					btVector3 pos;
					
					if (directionSign < 0)
					{
						rb = btRigidBody::upcast(obj1);
						pos = pt.m_positionWorldOnA;
					}
					else
					{
						rb = btRigidBody::upcast(obj0);
						pos = pt.m_positionWorldOnB;
					}	
					
					if(rb)
					{
						const muscleAnchor* newAnchor = new muscleAnchor(rb, pos, m_touchingNormal, false, true);
						m_anchors.push_back(newAnchor);
						
                        /*
						btScalar mass = rb->getInvMass() == 0 ? 0.0 : 1.0 / rb->getInvMass();
						btVector3 impulse = mass * dt * m_touchingNormal * getTension() / getActualLength() * -1.0* dist;
						rb->applyImpulse(impulse, pos);
                        */
					}
					
				}
			}
		}
	
	}
    
    std::sort (m_anchors.begin(), m_anchors.end(), m_ac);
    
    std::size_t n = m_anchors.size();
    for (int i = 0; i < n; i++)
    {
        std::cout << m_anchors[i]->getWorldPosition() << std::endl;
    }
    
}

MuscleNP::anchorCompare::anchorCompare(const muscleAnchor* m1, const muscleAnchor* m2) :
ma1(m1),
ma2(m2)
{
	
}

bool MuscleNP::anchorCompare::operator() (const muscleAnchor* lhs, const muscleAnchor* rhs)
{
   btVector3 pt1 = ma1->getWorldPosition();
   btVector3 ptN = ma2->getWorldPosition();
   
   btVector3 pt2 = lhs->getWorldPosition();
   btVector3 pt3 = rhs->getWorldPosition();
   
   btScalar lhDot = (ptN - pt1).dot(pt2);
   btScalar rhDot = (ptN - pt1).dot(pt3);
   
   return lhDot < rhDot;
}  

