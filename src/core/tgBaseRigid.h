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

#ifndef TG_BASE_RIGID_H
#define TG_BASE_RIGID_H

/**
 * @file tgBaseRigid.h
 * @brief Create a box shape as an obstacle or add it to your tensegrity
 * @author Brian Mirletz and Ryan Adams
 * $Id$
 */

// This application
#include "tgModel.h" // @todo: forward declare and move to tgRod.cpp (to be created)
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <vector>

// Forward declarations
class btRigidBody;

/**
 * A rod is a rigid body. Length is defined by nodes, radius and density
 * are defined by config.
 */
class tgBaseRigid : public tgModel
{
public:

    /** A class with a virtual memeber function requires a virtual destructor. */
    virtual ~tgBaseRigid();
    
    /**
     * Sets the btRigidBody to NULL
     */
    virtual void teardown();
    
    /**
     * Double dispatch funciton. Will pass itself and any children
     * back to the tgModelVisitor
     */
    virtual void onVisit(const tgModelVisitor& v) const;
    
    /**
     * Return the rod's mass in application-dependent units.
     * @return the rod's mass in application-dependent units
     */
    virtual double mass() const { return m_mass; }
    
    /**
     * Return the center of mass of the rod, a vector in 3-space.
     * @return the center of mass of the rod, a vector in 3-space
     */
    virtual btVector3 centerOfMass() const;

    /**
     * Getter for rigid body
     */
    virtual btRigidBody* getPRigidBody() 
    {
        return m_pRigidBody;
    }

    /**
     * Return the rod's orientation in Euler angles.
     * @return 3-vector of these euler angles
     */
    virtual btVector3 orientation() const;

protected:
	// Virtual base class
	tgBaseRigid(btRigidBody* pRigidBody,
		const tgTags& tags);

private:

    /** Integrity predicate. */
    bool invariant() const;

protected:
    
    /**
     * The Bullet Physics implementation of the rod.
     */
    btRigidBody* m_pRigidBody;
    
    /** The rod's mass. The units are application dependent. */
    const double m_mass;
    
};

#endif
