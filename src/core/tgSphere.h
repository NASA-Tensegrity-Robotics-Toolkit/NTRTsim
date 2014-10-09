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
 * A sphere is a rigid body. Its center is placed by tagging a node
 * and the radius is determined by its config struct.
 */
class tgSphere : public tgBaseRigid
{
public:

    /**
     * Holds two public member variables, density and radius, describing a
     * sphere configuration.
     * A constructor allows them to be set together and to default.
     */
    struct Config
    {
            /**
         * Initialize with radius and density, which may default.
         * @param[in] radius the sphere's radius; must be non-negative
         * @param[in] density the sphere's density; must be non-negative
         */
            Config(double r = 0.5,
                    double d = 1.0,
                    double f = 1.0,
                    double rf = 0.0,
                    double res = 0.2);



            /** The sphere's radius; must be nonnegative. */
            const double radius;

            /** The sphere's density; must be nonnegative. */
            const double density;
            
            /** The sphere's friction; 
             * must be greater than or equal to 0 */
            const double friction;

            /** The sphere's rolling friction; 
             * must be greater than or equal to 0 */
            const double rollFriction;
            
            /** The sphere's coefficient of restitution; 
             * must be between 0 and 1 (inclusive). */
            const double restitution;
    };
    
    /**
     * A container for a sphere shaped rigid body. Spheres are defined
     * at single nodes, and get their rigid bodies from the tgSphereInfo
     * class in tgcreator.
     */
        tgSphere(btRigidBody* pRigidBody,
                const tgTags& tags);
    
        /** A class with a virtual memeber function requires a virtual destructor. */
        virtual ~tgSphere();
    
    /**
     * Set the pointer to the rigid body to NULL, as it will be destroied
     * by the world.
     */
    virtual void teardown();
    
    virtual void onVisit(const tgModelVisitor& v) const;
    
private:

    /** Integrity predicate. */
    bool invariant() const;

};

#endif
