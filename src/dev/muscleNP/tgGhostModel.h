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

#ifndef TG_GHOST_MODEL_H
#define TG_GHOST_MODEL_H

/**
 * @file tgGhostModel.h
 * @brief Like tgBaseRigid, but uses btPairCachingGhostObject instead
 * @author Brian Mirletz
 * @date October 2014
 * $Id$
 */

// This application
#include "core/tgModel.h" 
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <vector>

// Forward declarations
class btPairCachingGhostObject;

/**
 * Holds a pointer to a btPairCachingGhostObject through the build process
 * It could be used to keep additional functions away from MuscleNP in the future,
 * by holding pointers to the dispatcher and similar
 */
class tgGhostModel : public tgModel
{
public:
	
	tgGhostModel(btPairCachingGhostObject* pCollisionObject,
		const tgTags& tags);
	
    /** A class with a virtual memeber function requires a virtual destructor. */
    virtual ~tgGhostModel();
    
    virtual void teardown();
    
    virtual void onVisit(const tgModelVisitor& v) const;
    
    /**
     * Getter for rigid body
     */
    virtual btPairCachingGhostObject* getPGhostObject() 
    {
        return m_pCollisionObject;
    }

private:

    /** Integrity predicate. */
    bool invariant() const;

protected:
    
    /**
     * The Bullet Physics implementation of the collision object.
     */
    btPairCachingGhostObject* m_pCollisionObject;
    
};

#endif
