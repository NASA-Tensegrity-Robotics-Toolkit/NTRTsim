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
#include "tgGhostModel.h"
#include "tgGhostInfo.h"

#include "tgcreator/tgUtil.h"
#include "core/muscleAnchor.h"
#include "core/tgCast.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorld.h"
#include "core/tgWorldBulletPhysicsImpl.h"

#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgNode.h"
#include "tgcreator/tgNodes.h"
#include "tgcreator/tgPair.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btQuickprof.h"

// The C++ Standard Library
#include <iostream>
#include <algorithm>    // std::sort, std::find_if
#include <cmath>
#include <stdexcept>

//#define VERBOSE
//#define MANIFOLD_CHECK

MuscleNP::MuscleNP(btPairCachingGhostObject* ghostObject,
 tgWorld& world,
 btRigidBody * body1,
 btVector3 pos1,
 btRigidBody * body2,
 btVector3 pos2,
 double coefK,
 double dampingCoefficient) :
Muscle2P (body1, pos1, body2, pos2, coefK, dampingCoefficient),
m_ghostObject(ghostObject),
m_world(world),
m_overlappingPairCache(tgBulletUtil::worldToDynamicsWorld(world).getBroadphase()),
m_dispatcher(tgBulletUtil::worldToDynamicsWorld(world).getDispatcher()),
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
#ifndef BT_NO_PROFILE 
    BT_PROFILE("calculateAndApplyForce");
#endif //BT_NO_PROFILE    
    
    pruneAnchors();
    
	updateAnchorList();
	
	m_rbForceMap.clear();
    m_rbForceScales.clear();
	
	const double tension = getTension();
    const double currLength = getActualLength();
    
    const double deltaStretch = currLength - m_prevLength;
    m_velocity = deltaStretch / dt;
    
    m_damping =  m_dampingCoefficient * m_velocity;
    
    if (btFabs(tension) * 1.0 < btFabs(m_damping))
    {
        m_damping =
          (m_damping > 0.0 ? tension * 1.0 : -tension * 1.0);
    }
    
    const double magnitude = tension + m_damping;
    
    // Apply forces
    std::size_t n = m_anchors.size();
    
    for (std::size_t i = 0; i < n; i++)
    {
        btVector3 force = btVector3 (0.0, 0.0, 0.0);
        if (i == 0)
        {
            btVector3 direction = m_anchors[i + 1]->getWorldPosition() - m_anchors[i]->getWorldPosition();
            force = direction.normalize() * magnitude;
        }
        // Will likely only be true for the last anchor
        else if (m_anchors[i]->sliding == false)
        {
            btVector3 direction = m_anchors[i]->getWorldPosition() - m_anchors[i - 1]->getWorldPosition();
            force = -direction.normalize() * magnitude;
        }
        else if (i < n - 1)
        {
			// Will fail if we already have this rigid body, but makes sure we're properly initialized otherwise
			m_rbForceMap.insert(std::pair<btRigidBody*, btVector3>(m_anchors[i]->attachedBody, btVector3(0.0, 0.0, 0.0)) );
			
            // Already normalized
            btVector3 direction = m_anchors[i]->getContactNormal();
            
			// Get normal to string
            btVector3 back = m_anchors[i - 1]->getWorldPosition(); 
            btVector3 current = m_anchors[i]->getWorldPosition(); 
            btVector3 forward = m_anchors[i + 1]->getWorldPosition(); 


			btVector3 first = (current - forward);
			btVector3 second = (current - back);
			
			btVector3 forceDir = (first + second).normalize();
			
			// Apply dot of contact normal with string's normal
			force = (tension * direction).dot(forceDir) * forceDir;
            
            // Only care about scaling sliding forces
            m_rbForceMap[m_anchors[i]->attachedBody] += force;

        }
        else
        {
            throw std::runtime_error("MuscleNP: First or last anchor is a sliding constraint!!");
        }
        
        m_anchors[i]->force = force;
         
    }
    
	std::map<btRigidBody*, btVector3>::iterator m_forceMapIt;
	
	for(m_forceMapIt = m_rbForceMap.begin(); m_forceMapIt != m_rbForceMap.end(); ++m_forceMapIt)
	{	
		btScalar totalForce = m_forceMapIt->second.length();
		btScalar forceScale = 1.0;
		
		// Might be able to come up with a more accurate than maximum. This is theoretical, but is a pretty special case
		if (totalForce > 2.0 * tension)
		{
			forceScale = 2.0 * tension / totalForce;
		}
		m_rbForceScales.insert(std::pair<btRigidBody*, btScalar> (m_forceMapIt->first, forceScale));
	}
    
    for (std::size_t i = 0; i < n; i++)
    {
		btRigidBody* body = m_anchors[i]->attachedBody;
		
		btVector3 contactPoint = m_anchors[i]->getRelativePosition();
		body->activate();
		
		btScalar forceScale = 1.0;
		// Scale the force of the sliding anchors
		if (m_anchors[i]->sliding)
		{
			forceScale = m_rbForceScales[body];
		}
		
		btVector3 impulse = m_anchors[i]->force * forceScale * dt;
		
		body->applyImpulse(impulse, contactPoint);
	}
    
    // Finished calculating, so can store things
    m_prevLength = currLength;
    
    // Do this last so the ghost object gets populated with collisions before it is deleted
    updateCollisionObject();
}

void MuscleNP::updateAnchorList()
{
    
#ifndef BT_NO_PROFILE 
    BT_PROFILE("updateAnchorList");
#endif //BT_NO_PROFILE      
    
	btManifoldArray	m_manifoldArray;
	btVector3 m_touchingNormal;
	
	// Only caches the pairs, they don't have a lot of useful information
	btBroadphasePairArray& pairArray = m_ghostObject->getOverlappingPairCache()->getOverlappingPairArray();
	int numPairs = pairArray.size();
    
    int numContacts = 2;
    
    m_anchorIt = m_anchors.begin() + 1;
    
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
		
		for (int j=0;j<m_manifoldArray.size();j++)
		{
			btPersistentManifold* manifold = m_manifoldArray[j];
			btScalar directionSign = manifold->getBody0() == m_ghostObject ? btScalar(-1.0) : btScalar(1.0);
            
			for (int p=0;p<manifold->getNumContacts();p++)
			{
				const btManifoldPoint& pt = manifold->getContactPoint(p);

				btScalar dist = pt.getDistance();
				
				if (dist < 0.0)
				{
					
					m_touchingNormal = pt.m_normalWorldOnB * directionSign;
					
                    btVector3 pos = directionSign < 0 ? pt.m_positionWorldOnB : pt.m_positionWorldOnA;
                    
					btRigidBody* rb = NULL;
					
					if (obj0 == m_ghostObject)
					{
						rb = btRigidBody::upcast(obj1);
					}
					else
					{
						rb = btRigidBody::upcast(obj0);
					}	
					
					if(rb)
					{  	
#ifdef MANIFOLD_CHECK
                        m_contactCheck = m_contactManifolds.insert(manifold);
                        if (m_contactCheck.second)
#endif
                        {
                    

							// Not permanent, sliding contact
							muscleAnchor* const newAnchor = new muscleAnchor(rb, pos, m_touchingNormal, false, true, manifold);
#if (1)							
							// Find position of new anchor
							while (m_anchorIt != (m_anchors.end() - 1) && m_ac.operator()((*m_anchorIt), newAnchor))
							{
								++m_anchorIt;
							}
							
							btVector3 pos0 = (*(m_anchorIt - 1))->getWorldPosition();
							btVector3 pos1 = newAnchor->getWorldPosition();
							btVector3 pos2 = (*m_anchorIt)->getWorldPosition();
							
							btVector3 lineA = (pos2 - pos1);
							btVector3 lineB = (pos0 - pos1);
							
							btScalar length1 = (pos0 - pos1).length();
							btScalar length2 = (pos1 - pos2).length();

							if (lineA.length() <= 0.1 || lineB.length() <= 0.1)
							{
								delete newAnchor;
							}
							else if((lineA).dot( newAnchor->getContactNormal()) < 0.0 ||
										(lineB).dot( newAnchor->getContactNormal()) < 0.0)
							{
								delete newAnchor;
							}
							else
							{	
													  
								m_anchorIt = m_anchors.insert(m_anchorIt, newAnchor);
								
								numContacts++;
							}
						}

#else
						
						m_anchorIt = m_anchors.insert(m_anchorIt, newAnchor);
						}
#endif
						
					}
					
				}
			}
		}
	
	}

// Sadly, even with the above insertion scheme, this is still a necessary sanity check
#if (1)    
    // Remove the permanaent anchors for sorting
    m_anchors.erase(m_anchors.begin());
    m_anchors.erase(m_anchors.end() - 1);
    
    std::sort (m_anchors.begin(), m_anchors.end(), m_ac);
    
    // Add these last to ensure we're in the right order

    m_anchors.insert(m_anchors.begin(), anchor1);
	m_anchors.insert(m_anchors.end(), anchor2);
#endif    
    //std::cout << "contacts " << numContacts << " unprunedAnchors " << m_anchors.size();
    
    //std::cout << " prunedAnchors " << m_anchors.size() << std::endl;
    
}

void MuscleNP::pruneAnchors()
{    
    // Find way to enter the loop without BS data
    int numPruned = 1;
    std::size_t i;
    
    // Attempt to eliminate points that would cause the string to push
    while (numPruned > 0)
    {
        #ifndef BT_NO_PROFILE 
            BT_PROFILE("pruneAnchors");
        #endif //BT_NO_PROFILE   
        numPruned = 0;
        i = 1;
        while (i < m_anchors.size() - 1)
        {
            btVector3 back = m_anchors[i - 1]->getWorldPosition(); 
            btVector3 current = m_anchors[i]->getWorldPosition(); 
            btVector3 forward = m_anchors[i + 1]->getWorldPosition(); 
            
            btVector3 lineA = (forward - current);
            btVector3 lineB = (back - current);
            
            btScalar normalValue1;
            btScalar normalValue2;
            
            btVector3 contactNormal = m_anchors[i]->getContactNormal();
            
            if (lineA.length() <= 0.0 || lineB.length() <= 0.0)
            {
                // Arbitrary value that deletes the nodes
                normalValue1 = -1.0;
                normalValue2 = -1.0;
            }
            else
            {
                //lineA.normalize();
                //lineB.normalize();
                //std::cout << "Normals " <<  std::btFabs(line.dot( m_anchors[i]->contactNormal)) << std::endl;
                normalValue1 = (lineA).dot(contactNormal);
                normalValue2 = (lineB).dot(contactNormal);
            }

            if ((normalValue1 < 0.0) || (normalValue2 < 0.0))
            {   
                #ifdef VERBOSE
                    std::cout << "Erased normal: " << normalValue1 << " "  << normalValue2 << " "; 
                #endif
                if (deleteAnchor(i))
				{
					numPruned++;
				}
            }
            else
            // Move anchor to best tangent position
            {
				if (!m_anchors[i]->permanent)
				{
					btVector3 tangentDir =( (lineB - lineA).cross(contactNormal)).normalize();
					btVector3 tangentMove = (lineB + lineA).dot(tangentDir) * tangentDir / 2.0;
					btVector3 newPos = current + tangentMove;
#if (1)
					// Check if new position is on body
					if (m_anchors[i]->setWorldPosition(newPos))
					{
						i++;
					}
					else
					{
						deleteAnchor(i);
						numPruned++;
					}
#else
					m_anchors[i]->setWorldPosition(newPos);
					i++;
#endif
				}
                else
                {
					i++;
				}
            }
        }
    }
    
    //std::cout << " Good Normal " << m_anchors.size();

#ifdef VERBOSE   
    std::size_t n = m_anchors.size();
    for (i = 0; i < n; i++)
    {      
        std::cout << m_anchors[i]->getWorldPosition() << std::endl;
    }
#endif
        
}

// This works ok at the moment. Need an effective way of determining if the rope is under an object
void MuscleNP::updateCollisionObject()
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("updateCollisionObject");
#endif //BT_NO_PROFILE    
    btDynamicsWorld& m_dynamicsWorld = tgBulletUtil::worldToDynamicsWorld(m_world);
    tgWorldBulletPhysicsImpl& bulletWorld =
      (tgWorldBulletPhysicsImpl&)m_world.implementation();
    
    m_dynamicsWorld.removeCollisionObject(m_ghostObject);
    // Consider managing this more locally
    btCollisionShape* shape = m_ghostObject->getCollisionShape();
    deleteCollisionShape(shape);
    delete m_ghostObject;
    
    btVector3 maxes(anchor2->getWorldPosition());
    btVector3 mins(anchor1->getWorldPosition());
    
    std::size_t n = m_anchors.size();
    
    for (std::size_t i = 0; i < n; i++)
    {
        btVector3 worldPos = m_anchors[i]->getWorldPosition();
        for (std::size_t j = 0; j < 3; j++)
        {
            if (worldPos[j] > maxes[j])
            {
                maxes[j] = worldPos[j];
            }
            if (worldPos[j] < mins[j])
            {
                mins[j] = worldPos[j];
            }
        }
    }
    btVector3 center = (maxes + mins)/2.0;
    
    btVector3 from = anchor1->getWorldPosition();
	btVector3 to = anchor2->getWorldPosition();
	
	btTransform transform = tgUtil::getTransform(from, to);
	
    transform.setOrigin(center);
    transform.setRotation(btQuaternion::getIdentity());
    
    btScalar radius = 0.001;

    btCompoundShape* m_compoundShape = new btCompoundShape(&m_world);
    
    for (std::size_t i = 0; i < n-1; i++)
    {
        btVector3 pos1 = m_anchors[i]->getWorldPosition();
        btVector3 pos2 = m_anchors[i+1]->getWorldPosition();
        
        btTransform t = tgUtil::getTransform(pos2, pos1);
        t.setOrigin(t.getOrigin() - center);
        
        btScalar length = (pos2 - pos1).length() / 2.0;
        if (length < radius)
        {
            
            //throw std::runtime_error("Teeny tiny contact object!!");
        }
        /// @todo - seriously examine box vs cylinder shapes
        btCylinderShape* box = new btCylinderShape(btVector3(radius, length, radius));
        
        m_compoundShape->addChildShape(t, box);
    }
    //m_compoundShape->setMargin(0.01);
    
    m_ghostObject = new btPairCachingGhostObject();
	
    m_ghostObject->setCollisionShape (m_compoundShape);
    m_ghostObject->setWorldTransform(transform);
    m_ghostObject->setCollisionFlags (btCollisionObject::CF_NO_CONTACT_RESPONSE);
    
    // @todo look up what the second and third arguments of this are
    m_dynamicsWorld.addCollisionObject(m_ghostObject,btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);

}

void MuscleNP::deleteCollisionShape(btCollisionShape* pShape)
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("deleteCollisionShape");
#endif //BT_NO_PROFILE
	
    if (pShape)
    {
		btCompoundShape* cShape = tgCast::cast<btCollisionShape, btCompoundShape>(pShape);
		if (cShape)
		{
			std::size_t n = cShape->getNumChildShapes();
			for( std::size_t i = 0; i < n; i++)
			{
				deleteCollisionShape(cShape->getChildShape(i));
			}
		}

        delete pShape;
    }
}

bool MuscleNP::deleteAnchor(int i)
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("deleteAnchor");
#endif //BT_NO_PROFILE 
    assert(i < m_anchors.size() && i >= 0);
#ifdef MANIFOLD_CHECK
    if (m_anchors[i]->manifold)
    {
        m_contactManifolds.erase(m_anchors[i]->getManifold());
    }
#endif
	if (m_anchors[i]->permanent != true)
	{
		delete m_anchors[i];
		m_anchors.erase(m_anchors.begin() + i);
		return true;
	}
	else
	{
		return false;
	}
}

MuscleNP::anchorCompare::anchorCompare(const muscleAnchor* m1, const muscleAnchor* m2) :
ma1(m1),
ma2(m2)
{
	
}

bool MuscleNP::anchorCompare::operator() (const muscleAnchor* lhs, const muscleAnchor* rhs) const
{
	return compareAnchors(lhs, rhs);
}  

#if (0)
void MuscleNP::anchorCompare::findPosition(std::vector<const muscleAnchor*> anchorList, std::vector<const muscleAnchor*>::Iterator it, const muscleAnchor* anc) const
{
	///@todo how do we ensure the iterator and the list are both the same??
	
	while(compareAnchors(*it, anc) && it != anchorList.end())
	{
		it++;
	}

}
#endif

bool MuscleNP::anchorCompare::compareAnchors(const muscleAnchor* lhs, const muscleAnchor* rhs) const
{
	// @todo make sure these are good anchors. Assert?
	   btVector3 pt1 = ma1->getWorldPosition();
	   btVector3 ptN = ma2->getWorldPosition();
	   
	   btVector3 pt2 = lhs->getWorldPosition();
	   btVector3 pt3 = rhs->getWorldPosition();
	   
	   btScalar lhDot = (ptN - pt1).dot(pt2);
	   btScalar rhDot = (ptN - pt1).dot(pt3);
	   
	   return lhDot < rhDot;

}
