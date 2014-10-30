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
#include <algorithm>    // std::sort
#include <cmath>
#include <stdexcept>

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
    
	updateAnchorList(dt);
	
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
            btVector3 direction = m_anchors[i]->contactNormal;
            
            // Law of cosines to get cos(angle)
            btVector3 back = m_anchors[i - 1]->getWorldPosition(); 
            btVector3 current = m_anchors[i]->getWorldPosition(); 
            btVector3 forward = m_anchors[i + 1]->getWorldPosition(); 
            
            btScalar Asqr = (forward - current).length2();
            btScalar A = (forward - current).length();
            btScalar Bsqr = (back - current).length2();
            btScalar B = (back - current).length();
            btScalar Csqr = (forward - back).length2();
            btScalar C = (forward - back).length();
            
           
            btScalar ang = btAcos((Asqr + Csqr - Bsqr) / (2.0 * A * C));
             // Cos(angle) * hyp = normal
            btScalar x = btSin(ang) * B;
            
            btScalar magnitude = (tension / A + tension / B) * x;
            
            magnitude = magnitude > tension ? tension : magnitude;
            
            force = direction * magnitude;
            
            
        }
        else
        {
            throw std::runtime_error("MuscleNP: First or last anchor is a sliding constraint!!");
        }
        
        btVector3 contactPoint = m_anchors[i]->getRelativePosition();
        m_anchors[i]->attachedBody->activate();
        m_anchors[i]->attachedBody->applyImpulse(force * dt, contactPoint);
    }
    
    // Finished calculating, so can store things
    m_prevLength = currLength;
    
    // Do this last so the ghost object gets populated with collisions before it is deleted
    updateCollisionObject();
}

void MuscleNP::updateAnchorList(double dt)
{
    
#ifndef BT_NO_PROFILE 
    BT_PROFILE("updateAnchorList");
#endif //BT_NO_PROFILE      
    
#if (0)
	std::vector<const muscleAnchor*>::iterator it = m_anchors.begin();
	
    for (it = m_anchors.begin(); it != m_anchors.end(); it++)
	{
		if ((*it)->permanent == false)
        {
            delete *it;
        }
	}
	
    m_anchors.clear();
#else
    
    // Remove the permanaent anchors for sorting
    m_anchors.erase(m_anchors.begin());
    m_anchors.erase(m_anchors.end() - 1);

#endif
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
						pos = pt.m_positionWorldOnB;
					}
					else
					{
						rb = btRigidBody::upcast(obj0);
						pos = pt.m_positionWorldOnA;
					}	
					
					if(rb)
					{
                        // Not permanent, sliding contact
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
    
    // Add these last to ensure we're in the right order

    m_anchors.insert(m_anchors.begin(), anchor1);
	m_anchors.insert(m_anchors.end(), anchor2);
    
    // Find way to enter the loop without BS data
    int numPruned = 1;
    std::size_t i;
    
    // Attempt to eliminate points that would cause the string to push
    while (numPruned > 0)
    {
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
            if (lineA.length() == 0.0 || lineB.length() == 0.0)
            {
                // Random value that deletes the nodes
                normalValue1 = -1.0;
                normalValue2 = -1.0;
                //std::cout << "Length tripped!" << std::endl;
            }
            else
            {
                lineA.normalize();
                lineB.normalize();
                //std::cout << "Normals " <<  std::btFabs(line.dot( m_anchors[i]->contactNormal)) << std::endl;
                normalValue1 = (lineA).dot( m_anchors[i]->contactNormal);
                normalValue2 = (lineB).dot( m_anchors[i]->contactNormal);
            }
            // Maybe change to double if Bullet uses double?
            if ((normalValue1 < 0.0) || (normalValue2 < 0.0))
            {   
                //std::cout << "Erased: " << normalValue1 << " "  << normalValue2 << " "; 
                delete m_anchors[i];
                m_anchors.erase(m_anchors.begin() + i);
                numPruned++;
            }      
            else
            {
                //std::cout << "Kept: " << normalValue1 << " "  << normalValue2 << " ";
                i++;
            }
            //std::cout << m_anchors.size() << " ";
            
        }
        
        //std::cout << "Pruned: " << numPruned << std::endl;
    }
   
    std::size_t n = m_anchors.size();
    for (i = 0; i < n; i++)
    {      
        //std::cout << m_anchors[i]->getWorldPosition(); 
#if (0)         
        if (i != 0 && i != n-1)
        {
            btVector3 back = m_anchors[i - 1]->getWorldPosition(); 
            btVector3 forward = m_anchors[i + 1]->getWorldPosition(); 
            
            btVector3 line = (forward - back).normalize();
            
            std::cout << " " <<  line.dot( m_anchors[i]->contactNormal);
        }   
#endif        
        //std::cout << std::endl;
    }

    
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
#if (0)    
    // @todo import this! Only the first two params matter
	tgBox::Config config(0.001, 0.001);

	tgStructure s;
	
	tgModel ectoplasm;

#if (1)	
    std::size_t n = m_anchors.size();
    for (std::size_t i = 0; i < n; i ++)
    {
        tgNode anchorPos = m_anchors[i]->getWorldPosition();
        s.addNode(anchorPos);
        if (i > 0)
        {
            s.addPair(i - 1, i, "box");
        }
    }
#else  
    tgNode from = anchor1->getWorldPosition();
	tgNode to = anchor2->getWorldPosition();
    
    s.addNode(from);
	s.addNode(to);
	
	s.addPair(0, 1, "box");
    
#endif // Single vs multi box methods
	
	tgBuildSpec spec;
	spec.addBuilder("box", new tgGhostInfo(config));
	
	// Create your structureInfo
	tgStructureInfo structureInfo(s, spec);
	// Use the structureInfo to build ourselves - this adds the new ghost object to the world
	structureInfo.buildInto(ectoplasm, m_world);
	
	std::vector<tgGhostModel*> m_hauntedHouse = tgCast::filter<tgModel, tgGhostModel> (ectoplasm.getDescendants());
	assert(m_hauntedHouse.size() > 0);

	m_ghostObject = m_hauntedHouse[0]->getPGhostObject();

    
#else // Stripped down version
    
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
        btBoxShape* box = new btBoxShape(btVector3(radius, length, radius));
        
        m_compoundShape->addChildShape(t, box);
    }
    m_ghostObject = new btPairCachingGhostObject();
	
    m_ghostObject->setCollisionShape (m_compoundShape);
    m_ghostObject->setWorldTransform(transform);
    m_ghostObject->setCollisionFlags (btCollisionObject::CF_NO_CONTACT_RESPONSE);
    
    // @todo look up what the second and third arguments of this are
    m_dynamicsWorld.addCollisionObject(m_ghostObject,btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);

#endif // Builder tools vs other method
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

MuscleNP::anchorCompare::anchorCompare(const muscleAnchor* m1, const muscleAnchor* m2) :
ma1(m1),
ma2(m2)
{
	
}

bool MuscleNP::anchorCompare::operator() (const muscleAnchor* lhs, const muscleAnchor* rhs) const
{
   btVector3 pt1 = ma1->getWorldPosition();
   btVector3 ptN = ma2->getWorldPosition();
   
   btVector3 pt2 = lhs->getWorldPosition();
   btVector3 pt3 = rhs->getWorldPosition();
   
   btScalar lhDot = (ptN - pt1).dot(pt2);
   btScalar rhDot = (ptN - pt1).dot(pt3);
   
   return lhDot < rhDot;
}  

