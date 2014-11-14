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
 * @file MuscleAnchor.cpp
 * @brief Definitions of class MuscleAnchor.
 * $Id$
 */
 
#include "muscleAnchor.h"

// The BulletPhysics library
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"

// The C++ Standard Library
#include <iostream>
#include <cassert>
#include <stdexcept>
 
muscleAnchor::muscleAnchor(btRigidBody * body,
               btVector3 worldPos,
               btVector3 cn,
               bool perm,
               bool slide,
               btPersistentManifold* m) :
  attachedBody(body),
  // Find relative position
  // This should give relative position in a default orientation.
  attachedRelativeOriginalPosition(attachedBody->getWorldTransform().inverse() *
                   worldPos),
  contactNormal(attachedBody->getWorldTransform().inverse().getBasis() * cn),
  height(999.0),
  permanent(perm),
  sliding(slide),
  force(0.0, 0.0, 0.0),
  manifold(m)
{
	assert(body);

	assert(manifold == NULL || body == manifold->getBody0() || body == manifold->getBody1());
}

muscleAnchor::~muscleAnchor()
{
    // World will delete attached body
    
}

// This returns current position relative to the rigidbody.
btVector3 muscleAnchor::getRelativePosition() const
{
    const btTransform tr = attachedBody->getWorldTransform();
    const btVector3 worldPos = tr * attachedRelativeOriginalPosition;
    return worldPos-this->attachedBody->getCenterOfMassPosition();
}

btVector3 muscleAnchor::getWorldPosition() const
{
    const btTransform tr = attachedBody->getWorldTransform();
    return tr * attachedRelativeOriginalPosition;
}

bool muscleAnchor::setWorldPosition(btVector3& newPos)
{
	/// @todo reverse this so we can get rid of a lot of else statements
	bool ret = true;
	//assert(manifold == NULL || attachedBody == manifold->getBody0() || attachedBody == manifold->getBody1());
	
	if (sliding)
	{
		bool useB = true;
		bool update = true;
		bool colCheck = true;
		if (manifold->getBody0() != attachedBody)
		{
			useB = false;			
		}
		if(!useB && manifold->getBody1() != attachedBody)
		{
			colCheck = false;
		}
		if (colCheck)
		{	
			btScalar length = INFINITY;
			
			
			int n = manifold->getNumContacts();
			
			btVector3 contactPos = getWorldPosition();
			btVector3 newNormal = contactNormal;
			btScalar dist = 0.0;
			for (int p = 0; p < n; p++)
			{
				const btManifoldPoint& pt = manifold->getContactPoint(p);
				
				// Original position picked at beginning
				btVector3 pos = useB ? pt.m_positionWorldOnA : pt.m_positionWorldOnB;
				
				btScalar contactDist = (pos - newPos).length();
				
				if (contactDist < length)
				{
					length = contactDist;
					contactPos = pos;
					
					btScalar directionSign = useB ? btScalar(1.0) : btScalar(-1.0);
					
					newNormal = attachedBody->getWorldTransform().inverse().getBasis() * pt.m_normalWorldOnB * directionSign;
					dist = pt.getDistance();
				
				}
				
			}
			
			// Assume we've lost this contact for some reason
			/*
			if (dist > 0.0 && length < 0.01)
			{
				ret = false;
			}
			*/
			
			if (length > 0.2)
			{
				// This makes contact handling better in some cases and worse in other
				// Better conservation of momentum without it, but contacts tend to exist a little too long
				//update = false;

			}
			if (update)
			{
				attachedRelativeOriginalPosition = attachedBody->getWorldTransform().inverse() *
						   newPos;
				
				if ((newNormal + contactNormal).length() < 0.1)
				{
					std::cout<< "Reversed normal" << std::endl;
					ret = false;
				}
				
				// Update again here in case we have the original manifold??
				if (length < 0.1)
				{
					contactNormal = newNormal;
				}
			}
			else if ((getWorldPosition() - contactPos).length() > 0.1)
			{
				ret = false;
			}
		}
		else
		{
			ret = false;
		}
		
		
	}
	else
	{
		ret = false;
		
		std::cerr << "Tried to update a static anchor" << std::endl;
		
	}
	
	return ret;
}

btVector3 muscleAnchor::getContactNormal() const
{

	const btTransform tr = attachedBody->getWorldTransform();
    btVector3 newNormal = (tr.getBasis() * contactNormal);
    newNormal = newNormal.length() > 0.0 ? newNormal.normalize() : btVector3(0.0, 0.0, 0.0);
    //assert(newNormal.length() == 1.0);
    return newNormal;

}

void muscleAnchor::updateManifold(btPersistentManifold* m)
{
	if (m && (m->getBody0() == attachedBody || m->getBody1() == attachedBody ))
	{
		std::pair<btScalar, btVector3> manifoldValues = getManifoldDistance(m);
		btScalar newDist = manifoldValues.first;
		if (!manifold)
		{
			manifold = m;
		}
		else if (getManifoldDistance(manifold).first >= newDist)
		{
			manifold = m;
		}
		
		if(manifold == m && newDist < 0.1)
		{
			//contactNormal = manifoldValues.second;
		}
	}
}

std::pair<btScalar, btVector3> muscleAnchor::getManifoldDistance(btPersistentManifold* m) const
{
	bool useB = true;
	bool colCheck = true;
	
	btScalar length = INFINITY;
	btVector3 newNormal = contactNormal;
	
	if (m->getBody0() != attachedBody)
	{
		useB = false;			
	}
	if(!useB && m->getBody1() != attachedBody)
	{
		colCheck = false;
	}
	if (colCheck)
	{	
	
					
		int n = m->getNumContacts();
		
		btVector3 contactPos = getWorldPosition();
		btScalar dist = 0.0;
		for (int p = 0; p < n; p++)
		{
			const btManifoldPoint& pt = m->getContactPoint(p);
			
			// Original position picked at beginning
			btVector3 pos = useB ? pt.m_positionWorldOnA : pt.m_positionWorldOnB;
			
			btScalar contactDist = (pos - getWorldPosition()).length();
			
			if (contactDist < length)
			{
				length = contactDist;
				contactPos = pos;
				
				btScalar directionSign = useB ? btScalar(1.0) : btScalar(-1.0);
				
				if (length < 0.1)
				{
					newNormal = attachedBody->getWorldTransform().inverse().getBasis() * pt.m_normalWorldOnB * directionSign;
				}
				
				dist = pt.getDistance();
				
				if (n >= 2)
				{
					std::cout << "Extra contacts!! " << dist << std::endl;
				}		
			}
		}
	}
	
	return std::make_pair<btScalar, btVector3> (length, newNormal);
}
