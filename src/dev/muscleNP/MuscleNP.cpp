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
 * @author Brian Mirletz
 * @date November 2014
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

// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btQuickprof.h"

// The C++ Standard Library
#include <iostream>
#include <algorithm>    // std::sort
#include <cmath>		// abs
#include <stdexcept>

//#define VERBOSE

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
m_world(world)
{

}
         
MuscleNP::~MuscleNP()
{
	btDynamicsWorld& m_dynamicsWorld = tgBulletUtil::worldToDynamicsWorld(m_world);
	m_dynamicsWorld.removeCollisionObject(m_ghostObject);
    // Consider managing this more locally
    btCollisionShape* shape = m_ghostObject->getCollisionShape();
    deleteCollisionShape(shape);
    delete m_ghostObject;
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

void MuscleNP::calculateAndApplyForce(double dt)
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("calculateAndApplyForce");
#endif //BT_NO_PROFILE    
    
    updateManifolds();
    
    pruneAnchors();
    
	updateAnchorList();
	
	pruneAnchors();

	m_forceTotals = btVector3(0.0, 0.0, 0.0);
    m_forceScales = btVector3(1.0, 1.0, 1.0);

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
			// Already normalized
            btVector3 direction = m_anchors[i]->getContactNormal();
            
			// Get normal to string
            btVector3 back = m_anchors[i - 1]->getWorldPosition(); 
            btVector3 current = m_anchors[i]->getWorldPosition(); 
            btVector3 forward = m_anchors[i + 1]->getWorldPosition(); 


			btVector3 first = (current - forward);
			btVector3 second = (current - back);
			
			btVector3 forceDir = (first.normalize()  + second.normalize() ).normalize();
		
			// Apply dot of contact normal with string's normal
			force = (tension * direction).dot(forceDir) * forceDir;
						
            // Only care about scaling sliding forces
            m_forceTotals += force;
        }
        else
        {
            throw std::runtime_error("MuscleNP: First or last anchor is a sliding constraint!!");
        }
        
        m_anchors[i]->force = force;
         
    }
    
	btVector3 maxForce = (anchor1->force + anchor2->force);
	
	for (std::size_t i = 0; i < 3; i++)
	{
		if (m_forceTotals[i] != maxForce[i] && m_forceTotals[i] != 0.0)
		{
			m_forceScales[i] = btFabs(maxForce[i] / m_forceTotals[i]);
		}
		else if (m_forceTotals[i] == 0.0)
		{
			m_forceScales[i] = 1.0;
		}
	} 

#ifdef VERBOSE
	std::cout << "Force Scaling " <<  m_forceScales << std::endl;
#endif

    for (std::size_t i = 0; i < n; i++)
    {
		btRigidBody* body = m_anchors[i]->attachedBody;
		
		btVector3 contactPoint = m_anchors[i]->getRelativePosition();
		body->activate();
	
		// Scale the force of the sliding anchors
		if (m_anchors[i]->sliding)
		{
			// Elementwise multiply
			m_anchors[i]->force *= m_forceScales;
		}

		btVector3 impulse = m_anchors[i]->force* dt;
		
		body->applyImpulse(impulse, contactPoint);
	}
    
    // Finished calculating, so can store things
    m_prevLength = currLength;
    
    // Do this last so the ghost object gets populated with collisions before it is deleted
    updateCollisionObject();
}

void MuscleNP::updateManifolds()
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("updateAnchorList");
#endif //BT_NO_PROFILE      
    
    // Copy this vector so we can remove as necessary
    
	btManifoldArray	m_manifoldArray;
	btVector3 m_touchingNormal;
	
	btBroadphaseInterface* const m_overlappingPairCache = tgBulletUtil::worldToDynamicsWorld(m_world).getBroadphase();
	
	// Only caches the pairs, they don't have a lot of useful information
	btBroadphasePairArray& pairArray = m_ghostObject->getOverlappingPairCache()->getOverlappingPairArray();
	int numPairs = pairArray.size();
    
    std::vector<muscleAnchor*> rejectedAnchors;
    
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
					
					//std::cout << pos << " " << manifold << " " << manifold->m_index1a << std::endl;
					
					if (manifold->getBody0() == m_ghostObject)
					{
						if (manifold->getBody1() == obj1)
							rb = btRigidBody::upcast(obj1);
						else if (manifold->getBody1() == obj0)
							rb = btRigidBody::upcast(obj0);
						else
						{
							throw std::runtime_error("Can't find the right object!!");
						}
					}
					else
					{
						if (manifold->getBody0() == obj0)
							rb = btRigidBody::upcast(obj0);
						else if (manifold->getBody0() == obj1)
							rb = btRigidBody::upcast(obj1);
						else
						{
							throw std::runtime_error("Can't find the right object!!");
						}
					}	
					
					if(rb)
					{  

						int anchorPos = findNearestPastAnchor(pos);
						assert(anchorPos < m_anchors.size() - 1);
						
						muscleAnchor* backAnchor = m_anchors[anchorPos];
						muscleAnchor* forwardAnchor = m_anchors[anchorPos + 1];
						
						btVector3 pos0 = backAnchor->getWorldPosition();
						btVector3 pos2 = forwardAnchor->getWorldPosition();
						
						btVector3 lineA = (pos2 - pos);
						btVector3 lineB = (pos0 - pos);
						
						btScalar lengthA = lineA.length();
						btScalar lengthB = lineB.length();
						
						// Not permanent, sliding contact
						muscleAnchor* const newAnchor = new muscleAnchor(rb, pos, m_touchingNormal, false, true, manifold);
						
						btVector3 contactNormal = newAnchor->getContactNormal();
									
						btScalar normalValue1 = (lineA).dot( newAnchor->getContactNormal()); 
						btScalar normalValue2 = (lineB).dot( newAnchor->getContactNormal()); 
						
						btScalar mDistB = backAnchor->getManifoldDistance(newAnchor->getManifold()).first;
						btScalar mDistA = forwardAnchor->getManifoldDistance(newAnchor->getManifold()).first;
						
						//std::cout << "Update Manifolds " << newAnchor->getManifold() << std::endl;
						
						bool del = false;					
						if (lengthB <= 0.1 && rb == backAnchor->attachedBody && mDistB < mDistA)
						{
							if(backAnchor->updateManifold(manifold))
								del = true;
								//std::cout << "UpdateB " << mDistB << std::endl;
						}
						if (lengthA <= 0.1 && rb == forwardAnchor->attachedBody && mDistA < mDistB)
						{
							if (forwardAnchor->updateManifold(manifold))
								del = true;
								//std::cout << "UpdateA " << mDistA << std::endl;
						}
						
						if (del)
						{
							/// @todo further examination of whether the anchors should be deleted here
							delete newAnchor;
						}
						else
						{
											
							m_newAnchors.push_back(newAnchor);
						} // If anchor passes distance tests
					} // If body is a rigid body
				} // If distance less than 0.0
			} // For number of contacts
		} // For number of manifolds
	} // For pairs of objects

	
}

void MuscleNP::updateAnchorList()
{
	int numContacts = 2;
    
	while (m_newAnchors.size() > 0)
	{
		// Not permanent, sliding contact
		muscleAnchor* const newAnchor = m_newAnchors[0];
		m_newAnchors.erase(m_newAnchors.begin());
		
		btVector3 pos1 = newAnchor->getWorldPosition();

		int anchorPos = findNearestPastAnchor(pos1);
		
		assert(anchorPos < m_anchors.size() - 1);
		
		muscleAnchor* backAnchor = m_anchors[anchorPos];
		muscleAnchor* forwardAnchor = m_anchors[anchorPos + 1];
		
		btVector3 pos0 = backAnchor->getWorldPosition();
		btVector3 pos2 = forwardAnchor->getWorldPosition();
			
		btVector3 lineA = (pos2 - pos1);
		btVector3 lineB = (pos0 - pos1);
		
		btScalar lengthA = lineA.length();
		btScalar lengthB = lineB.length();
		
		btVector3 contactNormal = newAnchor->getContactNormal();
						
		btScalar normalValue1 = (lineA).dot( newAnchor->getContactNormal()); 
		btScalar normalValue2 = (lineB).dot( newAnchor->getContactNormal()); 
		
		bool del = false;	
		
		btScalar mDistB = backAnchor->getManifoldDistance(newAnchor->getManifold()).first;
		btScalar mDistA = forwardAnchor->getManifoldDistance(newAnchor->getManifold()).first;
		
		//std::cout << "Update anchor list " << newAnchor->getManifold() << std::endl;
		
		// These may have changed, so check again				
		if (lengthB <= 0.1 && newAnchor->attachedBody == backAnchor->attachedBody && mDistB < mDistA)
		{
			if(backAnchor->updateManifold(newAnchor->getManifold()))
			{	
				del = true;
				//std::cout << "UpdateB " << mDistB << std::endl;
			}
		}
		if (lengthA <= 0.1 && newAnchor->attachedBody == forwardAnchor->attachedBody && mDistA < mDistB)
		{
			if(forwardAnchor->updateManifold(newAnchor->getManifold()))
				del = true;
				//std::cout << "UpdateA " << mDistA << std::endl;
		}

		if (del)
		{
			delete newAnchor;
		}
		else if(normalValue1 < 0.0 || normalValue2 < 0.0)
		{
			delete newAnchor;
		}
		else
		{		
			
			m_anchorIt = m_anchors.begin() + anchorPos + 1;
								  
			m_anchorIt = m_anchors.insert(m_anchorIt, newAnchor);
			
			numContacts++;
		}
	}
   
    //std::cout << "contacts " << numContacts << " unprunedAnchors " << m_anchors.size();
    
    //std::cout << " prunedAnchors " << m_anchors.size() << std::endl;
    
}

void MuscleNP::pruneAnchors()
{    
    int numPruned = 0;
    int passes = 0;
    std::size_t i;
    
    // Attempt to eliminate points that would cause the string to push
    while (numPruned > 0 || passes <= 3)
    {
        #ifndef BT_NO_PROFILE 
            BT_PROFILE("pruneAnchors");
        #endif //BT_NO_PROFILE   
        numPruned = 0;
        i = 1;
        while (i < m_anchors.size() - 1)
        {
			bool keep = true; //m_anchors[i]->updateContactNormal();
			
			numPruned = 0;
			btVector3 back = m_anchors[i - 1]->getWorldPosition(); 
			btVector3 current = m_anchors[i]->getWorldPosition(); 
			btVector3 forward = m_anchors[i + 1]->getWorldPosition(); 
			
			btVector3 lineA = (forward - current);
			btVector3 lineB = (back - current);
			

			btVector3 contactNormal = m_anchors[i]->getContactNormal();
			
			if (!m_anchors[i]->permanent)
			{
				btVector3 tangentDir = ( (lineB - lineA).cross(contactNormal)).normalize();
				btScalar tangentDot = (lineB + lineA).dot(tangentDir);
				btVector3 tangentMove = (lineB + lineA).dot(tangentDir) * tangentDir / 2.0;
				btVector3 newPos = current + tangentMove;
				// Check if new position is on body
				if (!keep || !m_anchors[i]->setWorldPosition(newPos))
				{
					deleteAnchor(i);
					numPruned++;
				}
			}
			else
			{
				i++;
			}
		
			if (numPruned == 0 && !m_anchors[i]->permanent)
			{
				btScalar normalValue1;
				btScalar normalValue2;
				
				// Get new values
				
				back = m_anchors[i - 1]->getWorldPosition(); 
				current = m_anchors[i]->getWorldPosition(); 
				forward = m_anchors[i + 1]->getWorldPosition(); 
			
				lineA = (forward - current);
				lineB = (back - current);
		
				contactNormal = m_anchors[i]->getContactNormal();
				
				
				if (lineA.length() < 0.05 || lineB.length() < 0.05)
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
					else
					{
						// Permanent anchor, move on
						i++;
					}
				}
				else
				{
					i++;
				}
			}
			
        }
        if (numPruned == 0)
        {
			passes++;
		}
		else
		{
			passes = 0;
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
	
	btDispatcher* m_dispatcher = tgBulletUtil::worldToDynamicsWorld(m_world).getDispatcher();
	btBroadphaseInterface* const m_overlappingPairCache = tgBulletUtil::worldToDynamicsWorld(m_world).getBroadphase();
	
    // Clear the existing child shapes
    btCompoundShape* m_compoundShape = tgCast::cast<btCollisionShape, btCompoundShape> (m_ghostObject->getCollisionShape());
    clearCompoundShape(m_compoundShape);
    
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
	
    btScalar radius = 0.001;

    for (std::size_t i = 0; i < n-1; i++)
    {
        btVector3 pos1 = m_anchors[i]->getWorldPosition();
        btVector3 pos2 = m_anchors[i+1]->getWorldPosition();
        
        // Children handles the orientation data
        btTransform t = tgUtil::getTransform(pos2, pos1);
        t.setOrigin(t.getOrigin() - center);
        
        btScalar length = (pos2 - pos1).length() / 2.0;
		
        /// @todo - seriously examine box vs cylinder shapes
        btCylinderShape* box = new btCylinderShape(btVector3(radius, length, radius));
        
        m_compoundShape->addChildShape(t, box);
    }
    //m_compoundShape->setMargin(0.01);
    
    btTransform transform;
    transform.setOrigin(center);
    transform.setRotation(btQuaternion::getIdentity());
    
    m_ghostObject->setCollisionShape (m_compoundShape);
    m_ghostObject->setWorldTransform(transform);
	
	// Delete the existing contacts in bullet to prevent sticking - may exacerbate problems with rotations
	m_overlappingPairCache->getOverlappingPairCache()->cleanProxyFromPairs(m_ghostObject->getBroadphaseHandle(),m_dispatcher);
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

void MuscleNP::clearCompoundShape(btCompoundShape* pShape)
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("clearCompoundShape");
#endif //BT_NO_PROFILE

	if (pShape)
	{
		while (pShape->getNumChildShapes() > 0)
		{
			btCollisionShape* pCShape = pShape->getChildShape(0);
			deleteCollisionShape(pCShape);
			pShape->removeChildShapeByIndex(0);
		}
	}
	
}

bool MuscleNP::deleteAnchor(int i)
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("deleteAnchor");
#endif //BT_NO_PROFILE 
    assert(i < m_anchors.size() && i >= 0);
	
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

int MuscleNP::findNearestPastAnchor(btVector3& pos)
{

	std::size_t i = 0;
	std::size_t n = m_anchors.size() - 1;
	assert (n >= 1);
	
	btScalar startDist = (pos - m_anchors[i]->getWorldPosition()).length();
	btScalar dist = startDist;
	
	while (dist <= startDist && i < n)
	{
		i++;
		btVector3 anchorPos = m_anchors[i]->getWorldPosition();
		dist = (pos - anchorPos).length();
		if (dist < startDist)
		{
			startDist = dist;
		}
	}
	
	// Check if we hit both break conditions at the same time
	if (dist > startDist)
	{
		i--;
	}
	
	if (i == n)
	{
		i--;
	}
	
	if (i == 0)
	{
		// Do nothing
	}
	else if (n > 1)
	{
		// Know we've got 3 anchors, so we need to compare along the line
		muscleAnchor* a0 = m_anchors[i - 1];
		muscleAnchor* an = m_anchors[i + 1];
		
		MuscleNP::anchorCompare m_acTemp(a0, an);
		
		btVector3 current = m_anchors[i]->getWorldPosition();
		
		m_acTemp.comparePoints(pos, current) ? i-- : i+=0;
		
		assert((m_anchors[i]->getWorldPosition() - pos).length() <= (a0->getWorldPosition() - pos).length()); 
	}
	muscleAnchor* prevAnchor = m_anchors[i];
	assert (prevAnchor);
	 
	return i; 

}

MuscleNP::anchorCompare::anchorCompare(const muscleAnchor* m1, const muscleAnchor* m2) :
ma1(m1),
ma2(m2)
{
	
}

bool MuscleNP::anchorCompare::operator() (const muscleAnchor* lhs, const muscleAnchor* rhs) const
{
	btVector3 pt2 = lhs->getWorldPosition();
	btVector3 pt3 = rhs->getWorldPosition();
	
	return comparePoints(pt2, pt3);
}  

bool MuscleNP::anchorCompare::comparePoints(btVector3& pt2, btVector3& pt3) const
{
	// @todo make sure these are good anchors. Assert?
	   btVector3 pt1 = ma1->getWorldPosition();
	   btVector3 ptN = ma2->getWorldPosition();
	   
	   btScalar lhDot = (ptN - pt1).dot(pt2);
	   btScalar rhDot = (ptN - pt1).dot(pt3);
	   
	   return lhDot < rhDot;

}
