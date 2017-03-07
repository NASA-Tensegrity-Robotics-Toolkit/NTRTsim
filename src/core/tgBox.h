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

#ifndef TG_BOX_H
#define TG_BOX_H

/**
 * @file tgBox.h
 * @brief Create a box shape as an obstacle or add it to your tensegrity
 * @author Brian Mirletz and Ryan Adams
 * $Id$
 */

// This application
#include "tgBaseRigid.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <vector>

// Forward declarations
//class btRigidBody;

/**
 * A box is a rigid body. Length is defined by nodes. Width, height and density
 * are defined by config.
 */
class tgBox : public tgBaseRigid
{
public:

    /**
     * Holds two public member variables, density and radius, describing a
     * box configuration.
     * A constructor allows them to be set together and to default.
     */
    struct Config
    {
        /**
         * Initialize with radius and density, which may default.
         * @param[in] radius the box's radius; must be non-negative
         * @param[in] density the box's density; must be non-negative
         */
            Config(double w = 1.0,
                    double h = 1.0,
                    double d = 1.0,
		    double f = 1.0,
                    double rf = 0.0,
                    double res = 0.2);


            /** The box's width; must be nonnegative. */
            const double width;

            /** The box's height; must be nonnegative. */
            const double height;
            
            /** The box's density; must be nonnegative. */
            const double density;
            
            /** The box's friction; 
             * must be greater than or equal to 0 */
            const double friction;

            /** The box's rolling friction; 
             * must be greater than or equal to 0 */
            const double rollFriction;
            
            /** The box's coefficient of restitution; 
             * must be between 0 and 1 (inclusive). */
            const double restitution;
    };
    
        tgBox(btRigidBody* pRigidBody,
                const tgTags& tags,
                const double length);
    
	/** A class with a virtual memeber function requires a virtual destructor. */
	virtual ~tgBox();

    virtual void teardown();
    
    virtual void onVisit(const tgModelVisitor& v) const;
    
    /**
     * Return the box's length in application-dependent units.
     * @return the box's length in application-dependent units
     */
    double length() const { return m_length; }


private:

    /** Integrity predicate. */
    bool invariant() const;

private:
    
    /** The box's length. The units are application dependent. */
    const double m_length;
};

#endif
