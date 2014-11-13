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

class btRigidBody;
class btPersistentManifold;

class muscleAnchor
{
public:
   

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
	
	btPersistentManifold* getManifold()
	{
		return manifold;
	}
	
	// Address should never be changed, body is not const
    btRigidBody * const attachedBody;
	
	// Store force so we can normalize it on a per-body basis
	btVector3 force;
	
    btScalar height;
    
    /**
     * A boolean value indicating where this anchor should be stored.
     * False implies it will be deleted after one update step
     */
    const bool permanent;
    
    /**
     * How the force is applied to the rigid body. True applies along the
     * contact normal, false is applied towards the next anchor
     */
    const bool sliding;
    
private:
	 // Relative to the body when it is first constructed
    btVector3 attachedRelativeOriginalPosition;
	
	// todo: write an accessor that asserts this is necessary and accurate
	btVector3 contactNormal;
	
	// todo: should this be const?
	btPersistentManifold* manifold;
};

#endif //NTRT_MUSCLE_ANCHOR_H_
