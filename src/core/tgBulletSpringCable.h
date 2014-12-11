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

#ifndef SRC_CORE_TG_BULLET_SPRING_CABLE_H_
#define SRC_CORE_TG_BULLET_SPRING_CABLE_H_

/**
 * @file tgBulletSpringCable.h
 * @brief Definitions of classes tgBulletSpringCable
 * $Id$
 */

// NTRT
#include "tgSpringCable.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library

#include <vector>

// Forward references
class btRigidBody;
class tgSpringCableAnchor;
class tgBulletSpringCableAnchor;

/**
 * This class defines the passive dynamics of a spring-cable system
 * in the Bullet physics engine
 * Formerly known as Muscle2P
 */
class tgBulletSpringCable : public tgSpringCable
{
public: 
    /**
     * The only constructor. Takes a list of anchors, a coefficient
     * of stiffness, a coefficent of damping, and optionally the amount
     * of pretension in the cable
     * @param[in] anchors - a list of this spring cable's attachements
     * @param[in] coefK - the stiffness of the spring. Must be positive
     * @param[in] dampingCoefficient - the damping in the spring. Must be non-negative
     * @param[in] pretension - must be small enough to keep the rest length positive
     */
    tgBulletSpringCable( const std::vector<tgBulletSpringCableAnchor*>& anchors,
                double coefK,
                double dampingCoefficient,
                double pretension = 0.0);
    
    /**
     * The virtual destructor. Deletes all of the anchors including anchor1 and anchor2
     */
    virtual ~tgBulletSpringCable();

    /**
     * Updates this object. Calls calculateAndApplyForce(dt)
     * @param[in] dt, must be positive
     */
    virtual void step(double dt);
    
    /**
     * Finds the distance between anchor1 and anchor2, and returns
     * the length between them
     */
    virtual const double getActualLength() const;
    
    /**
     * Returns the tension currently in the string by multiplying
     * the difference between the actual length and the rest length
     * by the stiffness coefficient
     */
    virtual const double getTension() const;
    
    /**
     * Returns a const vector of const anchors. Currently
     * casts from tgBulletSpringCableAnchors, which makes it impossible
     * to return a reference
     * @todo figure out how to cast and pass by reference
     */
    virtual const std::vector<const tgSpringCableAnchor*> getAnchors() const;
    
protected:
    
    /**
     * The list of contact points. tgBulletSpringCable typically has two
     * whereas tgBulletContactSpringCable will have more. 
     * Needs to be stored here for consistent rendering.
     * Vector has the convienence of tgCast functions, and we used to
     * need a random iterator to sort
     */
   std::vector<tgBulletSpringCableAnchor*> m_anchors;
    
    /**
     * The first attachement point for this spring cable. Storing it
     * seperately makes a number of functions easier
     */
    tgBulletSpringCableAnchor * const anchor1;

    /**
     * The other permanent attachment for this spring cable. 
     */
    tgBulletSpringCableAnchor * const anchor2;
    
private:
    
    /**
     * Calculates the current forces that need to be applied to 
     * the rigid bodies, and applies them to the bodies of anchor1 and 
     * anchor2
     */
    virtual void calculateAndApplyForce(double dt);

private: 
    /** Ensures integrity of member variables */
    bool invariant(void) const;
};

#endif  // SRC_CORE_TG_BULLET_SPRING_CABLE_H_
