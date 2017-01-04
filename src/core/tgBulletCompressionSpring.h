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

#ifndef SRC_CORE_TG_BULLET_COMPRESSION_SPRING_H_
#define SRC_CORE_TG_BULLET_COMPRESSION_SPRING_H_

/**
 * @file tgBulletCompressionSpring.h
 * @brief Definitions of class tgBulletCompressionSpring
 * @author Drew Sabelhaus, et al.
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <vector>

// Forward references
// Note that even though this object is not a tgSpringCable,
// we're still using tgSpringCableAnchor as a convenience.
// This works because there actually don't seem to be any references
// to tgSpringCable in tgSpringCableAnchor or tgBulletSpringCableAnchor.
class btRigidBody;
class tgSpringCableAnchor;
class tgBulletSpringCableAnchor;

/**
 * This class defines the passive dynamics of a compression spring
 * system, with damping, in the Bullet physics engine.
 */
class tgBulletCompressionSpring
{
public: 
    /**
     * The only constructor. Takes a list of anchors, a coefficient
     * of stiffness, a coefficent of damping, and rest length
     * of the spring.
     * @param[in] anchors - a list of this spring cable's attachements
     * @param[in] isFreeEndAttached - boolean flag.
     * If no, the spring only provides compression force. 
     * If yes, the spring is "attached" to both anchors, and provides a 
     * tension force too when the distance between anchors is greater 
     * than rest length.
     * @param[in] coefK - the stiffness of the spring. Must be positive
     * @param[in] dampingCoefficient - the damping in the spring. Must be non-negative.
     * @param[in] restLength - the length of the compression spring when unloaded.
     */
    tgBulletCompressionSpring( const std::vector<tgBulletSpringCableAnchor*>& anchors,
	        bool isFreeEndAttached,
                double coefK,
                double coefD,
                double restLength);
    
    /**
     * The virtual destructor. Deletes all of the anchors including 
     * anchor1 and anchor2
     */
    virtual ~tgBulletCompressionSpring();

    /**
     * Updates this object. Calls calculateAndApplyForce(dt)
     * @param[in] dt, must be positive
     */
    virtual void step(double dt);
    
    /**
     * Finds the distance between anchor1 and anchor2, and returns
     * the length between them
     */
    virtual const double getCurrentAnchorDistance() const;

    /**
     * Returns either restLength or current anchor distance, 
     * depending on isFreeEndAttached.
     */
    virtual const double getCurrentSpringLength() const;

    /**
     * Return the unit vector in the direction of this spring
     */
    virtual const btVector3 getAnchorDirectionUnitVector() const;

    /**
     * Return the location of the free end of the spring.
     * This is used, in particular, for rendering.
     * See tgBulletRenderer.
     */
    virtual const btVector3 getSpringEndpoint() const;

    
    /**
     * Returns the force currently in the spring, either compression only / a positive force only (if isFreeEndAttached is false), or potentially either + or - force (if isFreeEndAttached == true).
     */
    virtual const double getSpringForce() const;
    
    /**
     * Get the coefficent of stiffness
     */
    virtual const double getCoefK() const
    {
        return m_coefK;
    }
    
    /**
     * Get the coefficent of damping
     */
    virtual const double getCoefD() const
    {
        return m_coefD;
    }
    
    /**
     * Get the last change in length / time
     */
    virtual const double getVelocity() const
    {
        return m_velocity;
    }
    
    /**
     * Get the last value of the damping force
     */
    virtual const double getDampingForce() const
    {
        return m_dampingForce;
    }

    /**
     * Get the rest length of the spring
     * (we're using these accessor functions per C++ style guidelines.)
     */
    virtual const double getRestLength() const
    {
        return m_restLength;
    }

    /**
     * Return the boolean: is the free end attached?
     */
    virtual const bool isFreeEndAttached() const
    {
        return m_isFreeEndAttached;
    }
    
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

    /**
     * The force in the spring due to damping, at the last update step. 
     * Stored so we can get it without passing a dt
     */
    double m_dampingForce;
    
    /**
     * The velocity of the spring tip at the last update step. Stored so we
     * can get it without passing a dt
     */
    double m_velocity;

    /**
     * Boolean flag controlling the application of either tension forces or not.
     */
    bool m_isFreeEndAttached;

    /**
     * The stiffness coefficient
     * Units of mass / sec ^2
     * Must be positive
     */
    const double m_coefK;

    /**
     * The damping coefficient.
     * Units of mass / sec. 
     * Must be non-negative
     */
    const double m_coefD;
    
    /**
     * The rest length of the spring. Must be non negative
     */
    double m_restLength;
 
    /**
     * The previous actual length of the spring. Used when calculating
     * force and velocity
     */
    double m_prevLength;
    
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
