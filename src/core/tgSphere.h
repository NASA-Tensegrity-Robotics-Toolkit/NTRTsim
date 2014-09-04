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

#ifndef TG_SPHERE_H
#define TG_SPHERE_H

/**
 * @file tgSphere.h
 * @brief Contains the definition of class tgSphere
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
class btRigidBody;

/**
 * A rod is a rigid body. Length is defined by nodes, radius and density
 * are defined by config.
 */
class tgSphere : public tgBaseRigid
{
public:

    /**
     * Holds two public member variables, density and radius, describing a
     * rod configuration.
     * A constructor allows them to be set together and to default.
     */
    struct Config
    {
            /**
         * Initialize with radius and density, which may default.
         * @param[in] radius the rod's radius; must be non-negative
         * @param[in] density the rod's density; must be non-negative
         */
            Config(double r = 0.5,
                    double d = 1.0,
                    double f = 1.0,
                    double rf = 0.0,
                    double res = 0.2);



            /** The rod's radius; must be nonnegative. */
            const double radius;

            /** The rod's density; must be nonnegative. */
            const double density;
            
            /** The rod's friction; 
             * must be between 0 and 1 (inclusive). */
            const double friction;

            /** The rod's rolling friction; 
             * must be between 0 and 1 (inclusive). */
            const double rollFriction;
            
            /** The rod's coefficient of restitution; 
             * must be between 0 and 1 (inclusive). */
            const double restitution;
    };
    
        tgSphere(btRigidBody* pRigidBody,
                const tgTags& tags);
    
        /** A class with a virtual memeber function requires a virtual destructor. */
        virtual ~tgSphere();
    
    virtual void teardown();
    
    virtual void onVisit(const tgModelVisitor& v) const;
    
private:

    /** Integrity predicate. */
    bool invariant() const;

};

#endif
