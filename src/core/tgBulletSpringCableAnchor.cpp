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
 * @file tgBulletSpringCableAnchor.cpp
 * @brief Definitions of class tgBulletSpringCableAnchor.
 * $Id$
 */
 
#include "tgBulletSpringCableAnchor.h"

// The BulletPhysics library
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"

// The C++ Standard Library
#include <iostream>
#include <cassert>
#include <stdexcept>

// Does the contact normal get updated using the body's rotation? (99% sure its yes)
#define USE_BASIS
// Do we update the contact based on the manifold? - Causes contacts to be missed, doesn't prevent angular energy from accumulating
// Contact update also appears to magnify force direction issues
#define SKIP_CONTACT_UPDATE
//#define VERBOSE

tgBulletSpringCableAnchor::tgBulletSpringCableAnchor(btRigidBody * body,
               btVector3 worldPos,
               btVector3 cn,
               bool perm,
               bool slide,
               btPersistentManifold* m) :
  tgSpringCableAnchor(worldPos, cn, perm, slide),
  attachedBody(body),
  // Find relative position
  // This should give relative position in a default orientation.
  attachedRelativeOriginalPosition(attachedBody->getWorldTransform().inverse() *
                   worldPos),
#ifdef USE_BASIS
  contactNormal(attachedBody->getWorldTransform().inverse().getBasis() * cn),
#else
  contactNormal(cn),
#endif
  manifold(m)
{
	assert(body);
	
	// Ensure we're either not using a manifold, or we currently have the right manifold
	assert(manifold == NULL || body == manifold->getBody0() || body == manifold->getBody1());
}

tgBulletSpringCableAnchor::~tgBulletSpringCableAnchor()
{
    // World will delete attached body, bullet owns the manifolds
    
}

// This returns current position relative to the rigidbody.
btVector3 tgBulletSpringCableAnchor::getRelativePosition() const
{
    const btTransform tr = attachedBody->getWorldTransform();
    const btVector3 worldPos = tr * attachedRelativeOriginalPosition;
    return worldPos-this->attachedBody->getCenterOfMassPosition();
}

btVector3 tgBulletSpringCableAnchor::getWorldPosition() const
{
    const btTransform tr = attachedBody->getWorldTransform();
    return tr * attachedRelativeOriginalPosition;
}

bool tgBulletSpringCableAnchor::setWorldPosition(btVector3& newPos)
{
	bool ret = false;

	// Only sliding anchors should have their positions changed
	if (sliding)
	{
		/// @todo - this is very similar to getManifoldDistance. Is there a good way to combine them??
		// Figure out which body to use
		bool useB = true;
		if (manifold->getBody0() != attachedBody)
		{
			useB = false;			
		}
		
		if(useB || manifold->getBody1() == attachedBody)
		{	
			btScalar length = INFINITY;
			
			int n = manifold->getNumContacts();
			
			btVector3 contactPos = getWorldPosition();
			btVector3 newNormal = contactNormal;
			btScalar dist = 0.0;
			
			// Find closest contact point in this manifold
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
					
#ifdef USE_BASIS
					newNormal = attachedBody->getWorldTransform().inverse().getBasis() * pt.m_normalWorldOnB * directionSign;
#else
					newNormal = pt.m_normalWorldOnB * directionSign;
#endif
					dist = pt.getDistance();
					
#ifdef VERBOSE
					if (n >= 2)
					{
						std::cout << "Extra contacts!! " << p << " " << dist << std::endl;
					}
#else
                    // Suppress compiler warning for unused variable
                    (void) dist;
#endif
				}
				
			}
			
			// We've lost this contact for some reason, skip update and delete
			//if (!(dist > 0.0 && length < 0.01))
			{	
				// If contact is sufficiently close, update
				if (length < 0.1)
				{
					// This makes contact handling better in some cases and worse in other
					// Better conservation of momentum without it, but contacts tend to exist a little too long
					// Just deleting at this stage is better for sliding, but worse for contact with multiple bodies
					attachedRelativeOriginalPosition = attachedBody->getWorldTransform().inverse() *
							   newPos;
					
					if ((newNormal + contactNormal).length() < 0.5)
					{
                        #ifdef VERBOSE
						std::cout<< "Reversed normal" << std::endl;
                        #endif
					}
					else
					{
						ret = true;
					}
					
	// Update again here in case we have the original manifold??			
	#ifndef SKIP_CONTACT_UPDATE
					contactNormal = newNormal;
	#endif	
				}
				// Check if the update was bad based on the original position, if not delete
				else if ( (getWorldPosition() - contactPos).length() <= 0.1)
				{
					ret = true;
				}
			}
		}
		else
        {
#ifdef VERBOSE
            std::cout << "Manifold out of date!" << std::endl;
#endif
        }
		// Else: neither body is attached, delete
	}
	else
	{
		std::cerr << "Tried to update a static anchor" << std::endl;
		// This will return as a delete, make sure you check the anchor is not permanent		
	}
	
	return ret;
}

btVector3 tgBulletSpringCableAnchor::getContactNormal() const
{

#ifdef USE_BASIS
	const btTransform tr = attachedBody->getWorldTransform();
    btVector3 newNormal = (tr.getBasis() * contactNormal);
    newNormal = newNormal.length() > 0.0 ? newNormal.normalize() : btVector3(0.0, 0.0, 0.0);
    //assert(newNormal.length() == 1.0);
    return newNormal;
#else
	return contactNormal;
#endif

}

bool tgBulletSpringCableAnchor::updateManifold(btPersistentManifold* m)
{
	bool ret = false;
	// Does the new manifold actually affect the attached body
	if (m && (m->getBody0() == attachedBody || m->getBody1() == attachedBody ))
	{
		std::pair<btScalar, btVector3> manifoldValues = getManifoldDistance(m);
		btScalar newDist = manifoldValues.first;
		// If the original manifold is NULL, just use the new one
		if (!manifold)
		{
			//manifold = m;
#ifdef VERBOSE
            std::cout << "Old manifold was NULL" << std::endl;
#endif
			ret = true;
		}
		// Use new manifold
		else if (getManifoldDistance(manifold).first >= newDist)
		{
			//manifold = m;
			ret = true;
		}
		
		// If we updated, ensure the new contact normal is good
		if(ret)
		{
			btVector3 newNormal = manifoldValues.second;
			if ((newNormal + contactNormal).length() < 0.5)
			{
                #ifdef VERBOSE
				 std::cout <<"Reversed normal during anchor update" << std::endl;
                #endif
				 ret = false;
			}
			else
			{
				manifold = m;
			}
			#ifndef SKIP_CONTACT_UPDATE
			// Updating here appears to break conservation of momentum
			//contactNormal = newNormal;
			#endif
		}
	}
#ifdef VERBOSE
	if (!ret)
    {
        std::cout << "Failed to update manifold!" << std::endl;
    }
#endif
	
	return ret;
}

std::pair<btScalar, btVector3> tgBulletSpringCableAnchor::getManifoldDistance(btPersistentManifold* m) const
{
	bool useB = true;
	
	btScalar length = INFINITY;
	btVector3 newNormal = contactNormal;
	
    if (!permanent)
    {
        if (m->getBody0() != attachedBody)
        {
            useB = false;			
        }
        if(useB || m->getBody1() == attachedBody)
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
                        #ifdef USE_BASIS
                        newNormal = attachedBody->getWorldTransform().inverse().getBasis() * pt.m_normalWorldOnB * directionSign;
                        #else
                        newNormal = pt.m_normalWorldOnB * directionSign;
                        #endif
                    }
                    
                    dist = pt.getDistance();
    #ifdef VERBOSE				
                    if (n >= 2)
                    {
                        std::cout << "Extra contacts!! " << p << " length " << length << " dist: " << dist << std::endl;
                    }
    #else
                        // Suppress compiler warning for unused variable
                        (void) dist;
    #endif
                }
            }
        }
    }
	
	return std::make_pair<btScalar, btVector3> (length, newNormal);
}
