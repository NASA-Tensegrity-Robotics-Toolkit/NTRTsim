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

#ifndef SRC_CORE_TG_BULLET_SPRING_CABLE_ANCHOR_H_
#define SRC_CORE_TG_BULLET_SPRING_CABLE_ANCHOR_H_

/**
 * @file tgBulletSpringCableAnchor.h
 * @brief Definitions of class tgBulletSpringCableAnchor, formerly muscleAnchor.
 * @author Brian Mirletz and Atil Iscen
 * @date November 2014
 * $Id$
 */

// This library
#include "core/tgSpringCableAnchor.h"

// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <string>
#include <utility> //std::pair

// Forward References
class btRigidBody;
class btPersistentManifold;
class tgBulletContactSpringCable;

/**
 * A class that allows tgBulletSpringCable and tgBulletContactSpringCable to attach to btRigidBodies
 * Anchors track a specific point on a body as that body translates
 * and rotates. They can either be 'non-sliding' which typically means
 * a pin jointed anchor (and are typically permanent), or sliding,
 * which means they track a specific contact point within a
 * btPersistentManifold. 
 */
class tgBulletSpringCableAnchor : public tgSpringCableAnchor
{
public:
	// tgBulletContactSpringCable needs to scale the forces
   friend class tgBulletContactSpringCable;
	
	/**
	 * The only constructor. At a minimum requires a body and a position
	 * on that body to track. Sliding anchors require additional data
	 * @param[in] body - a pointer to the btRigidBody this is attached to
	 * @param[in] pos - The position in world coordinates where this attaches
	 * @param[in] cn - A btVector3 that specifies the direction of contact
	 * Only requried for sliding anchors
	 * @param[in] perm - Whether or not this anchor can be deleted in
	 * the middle of a simulation
	 * @param[in] slide - Whether this represents a pin joint or a sliding contact
	 * @param[in] m - a btPersistenManifold that is used to track contacts
	 */
    tgBulletSpringCableAnchor(btRigidBody *body, 
					btVector3 pos,
					btVector3 cn = btVector3(0.0, 0.0, 0.0),
					bool perm = true, 
					bool slide = false,
					btPersistentManifold* m = NULL);
    
    /**
     * Destructor, nothing to delete
     */
    virtual ~tgBulletSpringCableAnchor();
    
    /**
     * Return the current position of the anchor in world coordinates
     * Uses attachedRelativeOriginalPosition and the attachedBody's
     * btTransform
     */
    virtual btVector3 getWorldPosition() const;
	
	/**
	 * Update attachedRelativeOriginalPosition based on the sliding
	 * of the string. This also checks if the new sliding position
	 * is still on the body.
	 * @return bool returns if this point is actually on the body. The
	 * body should be deleted if this returns false
	 */
	virtual bool setWorldPosition(btVector3& newPos);
	
    /**
     * Get the position of the point in body coordinaates
     * @return a btVector3 in body coordinates
     */
    virtual btVector3 getRelativePosition() const;
    
    /**
     * Return an up to date contact normal based on the rigid
     * body's btTransform
     * @return the contact normal, accounting for any rotation from
     * when the body was first contacted.
     */
    virtual btVector3 getContactNormal() const;
    
    /**
     * Update our manifold pointer. This memory is often reassigned
     * so the new manifold is accepted if our old manifold no longer
     * contains our rigid body. It is also accepted if its contact point
     * is closer than our manifold's
     * @return a bool that is true if the new manifold was accepted
     */
    bool updateManifold(btPersistentManifold* m);
	
	/**
	 * Return a pointer to our btPersistentManifold (typically so 
	 * something can compare getManifoldDistance with a new manifold)
	 */
	btPersistentManifold* getManifold() const
	{
		return manifold;
	}
	
	/**
	 * The rigid body we affect
	 * Address should never be changed, body is not const
	 * @todo Create applyForce functions so this doesn't have to
	 * be exposed
     */
    btRigidBody * const attachedBody;
  
    /**
     * A pair of the distance between the current world position and 
     * this manifolds contact point, as well as the contact normal
     * of this manifold
     * @return distance to the contact, contact normal 
     */
    std::pair<btScalar, btVector3> getManifoldDistance(btPersistentManifold* m) const;
    
private:
	
   /**
	 * The relative world position, stored by multiplying by
	 * the inverse transform of the initial orientation
	 */
    btVector3 attachedRelativeOriginalPosition;
	
	/**
	 * For sliding anchors: the normal between the relevent two
	 * btCollisionObjects. Used in determining validity of anchors and
	 * applying forces to rigid bodies
	 */
	btVector3 contactNormal;
	
	/**
	 * The manifold that generated this body if it is a sliding contact
	 * NULL if its a pin joint
	 * Not const, bullet owns this, and we update it as best we can
	 */
	btPersistentManifold* manifold;
	
};

#endif // SRC_CORE_TG_BULLET_SPRING_CABLE_ANCHOR_H_
