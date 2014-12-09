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

#ifndef SRC_CORE_TGSPRINGCABLEANCHOR_H_
#define SRC_CORE_TGSPRINGCABLEANCHOR_H_

/**
 * @file tgSpringCableAnchor.h
 * @brief Definitions of class tgSpringCableAnchor.
 * @author Brian Mirletz and Atil Iscen
 * @date November 2014
 * $Id$
 */

// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// The C++ Standard Library

#include <utility> //std::pair


/**
 * A class that allows attaches tgSpringCables to rigid bodies. Only
 * dependency on Bullet is btScalar and btVector3 (linear algebra)
 */
class tgSpringCableAnchor
{
public:

	/**
	 * The only constructor. At a minimum requires a body and a position
	 * on that body to track. Sliding anchors require additional data
	 * @param[in] pos - The position in world coordinates where this attaches
	 * @param[in] cn - A btVector3 that specifies the direction of contact
	 * Only requried for sliding anchors
	 * @param[in] perm - Whether or not this anchor can be deleted in
	 * the middle of a simulation
	 * @param[in] slide - Whether this represents a pin joint or a sliding contact
	 * @note this does noth
	 */
    tgSpringCableAnchor(btVector3 pos,
					btVector3 cn = btVector3(0.0, 0.0, 0.0),
					bool perm = true, 
					bool slide = false) :
	permanent(perm),
	sliding(slide),
	force(0.0, 0.0, 0.0)
	{
		// Suppress unused variables, since these require btTransforms to be stored properly
		(void) pos;
		(void) cn;
	}
    
    /**
     * Virtual destructor, nothing to delete
     */
    virtual ~tgSpringCableAnchor() { }
    
    /**
     * Return the current position of the anchor in world coordinates
     */
    virtual btVector3 getWorldPosition() const = 0;
	
	/**
	 * Update attachedRelativeOriginalPosition based on the sliding
	 * of the string. This also checks if the new sliding position
	 * is still on the body.
	 * @return bool returns if this point is actually on the body. The
	 * body should be deleted if this returns false
	 */
	virtual bool setWorldPosition(btVector3& newPos) = 0;
	
    /**
     * Get the position of the point in body coordinaates
     * @return a btVector3 in body coordinates
     */
    virtual btVector3 getRelativePosition() const = 0;
    
    /**
     * Return an up to date contact normal based on the rigid
     * body's btTransform
     * @return the contact normal, accounting for any rotation from
     * when the body was first contacted.
     */
    virtual btVector3 getContactNormal() const = 0;
    
	/**
	 * @return a const reference to the force we just applied (or are 
	 * about to apply)
	 */
	const btVector3& getForce() const
	{
		return force;
	}

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
      
protected:
	
	/**
	 * Store force so we can normalize it appropreately. Set and
	 * accessed directly by MuscleNP
	 */
	btVector3 force;
	
};

#endif // SRC_CORE_TGSPRINGCABLEANCHOR_H_
