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

#ifndef NTRT_MUSCLE_ANCHOR_H_
#define NTRT_MUSCLE_ANCHOR_H_

/**
 * @file MuscleAnchor.h
 * @brief Definitions of class MuscleAnchor.
 * $Id$
 */

// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <string>
#include <utility> //std::pair

class btRigidBody;
class btPersistentManifold;
class MuscleNP;

class muscleAnchor
{
public:
	// MuscleNP needs to scale the forces
   friend class MuscleNP;

    muscleAnchor(btRigidBody *body, 
					btVector3 pos,
					btVector3 cn = btVector3(0.0, 0.0, 0.0),
					bool perm = true, 
					bool slide = false,
					btPersistentManifold* m = NULL);
    
    ~muscleAnchor();
    
    btVector3 getWorldPosition() const;
	
	// Bool returns if this point is actually on the body
	bool setWorldPosition(btVector3& newPos);
	
    // Relative to the body
    btVector3 getRelativePosition() const;
    
    btVector3 getContactNormal() const;
    
    void updateManifold(btPersistentManifold* m);
	
	btPersistentManifold* getManifold() const
	{
		return manifold;
	}
	
	btVector3 getForce() const
	{
		return force;
	}

	// Address should never be changed, body is not const
    btRigidBody * const attachedBody;
	
	/// @todo remove this. Anchors and sensors should be seperate classes
    btScalar height;
    
    /**
     * A boolean value indicating whether this a temporary or permanent contact
     * if permanent do not delete it until teardown!!
     */
    const bool permanent;
    
    /**
     * How the force is applied to the rigid body. True applies along the
     * contact normal, false is applied towards the next anchor.
     * Application depends on other classes
     * @todo Do we want an internal apply force function? May simplify things (prevent Muscles from needing to include rigid bodies??)
     */
    const bool sliding;
    
private:

	std::pair<btScalar, btVector3> getManifoldDistance(btPersistentManifold* m) const;
	 // Relative to the body when it is first constructed
    btVector3 attachedRelativeOriginalPosition;
	
	btVector3 contactNormal;
	
	// Not const, bullet owns this
	btPersistentManifold* manifold;
	
	// Store force so we can normalize it on a per-body basis
	btVector3 force;
	
};

#endif //NTRT_MUSCLE_ANCHOR_H_
