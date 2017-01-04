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

#ifndef SRC_CORE_TG_BULLET_UNIDIR_COMPR_SPR_H_
#define SRC_CORE_TG_BULLET_UNIDIR_COMPR_SPR_H_

/**
 * @file tgBulletUnidirComprSpr.h
 * @brief Definitions of class tgBulletUnidirComprSpr,
 *    a version of tgBulletCompressionSpring that only
 *    applies a force in one direction ("unidirectional.")
 * @author Drew Sabelhaus, et al.
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <vector>

// This class inherits from tgBulletCompressionSpring
#include "tgBulletCompressionSpring.h"

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
 * It extends tgBulletCompressionSpring, and changes the way that forces
 * are applied: instead of calculating and applying force along the vector
 * between the two anchor points, it only calculates/applies force along
 * one of the axes: X, Y or Z.
 * So for example, if it's only in Z, one anchor could move around in X or Y
 * and the applied force wouldn't change.
 * Here, we use "Unidir" to mean "Unidirectional."
 */
class tgBulletUnidirComprSpr : public tgBulletCompressionSpring
{
public: 
    /**
     * The only constructor. Takes a list of anchors, a flag controlling
     * attached-ness of the free end of the spring, a coefficient
     * of stiffness, a coefficent of damping, rest length
     * of the spring, and a btVector3 in one of the X, Y, or Z directions.
     * @param[in] anchors - a list of this spring cable's attachements
     * @param[in] isFreeEndAttached - boolean flag.
     * If no, the spring only provides compression force. 
     * If yes, the spring is "attached" to both anchors, and provides a 
     * tension force too when the distance between anchors is greater 
     * than rest length.
     * @param[in] coefK - the stiffness of the spring. Must be positive
     * @param[in] dampingCoefficient - the damping in the spring. Must be non-negative.
     * @param[in] restLength - the length of the compression spring when unloaded.
     * @param[in] direction - the direction of the force to be applied, a btVector3.
     * the current version will only support (1,0,0), (0,1,0), or (0,0,1), or
     * the negatives of those vectors.
     * NOTE that if the To anchor is in the negative direction with respect to the
     * From anchor, then direction should be NEGATIVE.
     * NOTE that although direction is a pointer, it is not deleted in this class 
     * or any of the other related classes in core. That is because
     * it's stored in the const Config struct of an application: it's only 
     * created once, not dynamically, so it's only deleted at the very 
     * end of the application (NOT during any individual setups or teardowns.)
     */
    tgBulletUnidirComprSpr(
		const std::vector<tgBulletSpringCableAnchor*>& anchors,
	        bool isFreeEndAttached,
                double coefK,
                double coefD,
                double restLength,
		btVector3 * direction);
    
    /**
     * The virtual destructor. Deletes all of the anchors including 
     * anchor1 and anchor2
     */
    virtual ~tgBulletUnidirComprSpr();

    /**
     * Updates this object. Calls calculateAndApplyForce(dt)
     * @param[in] dt, must be positive
     */
    virtual void step(double dt);

    /**
     * Returns the distance between anchors along the vector m_direction.
     * This function just calculates the dot product between m_direction and
     * getCurrentAnchorDistance from the parent class.
     */
    virtual const double getCurrentAnchorDistanceAlongDirection() const;
    
    /**
     * Returns either restLength or the length of the spring along the 
     * single direction, depending on isFreeEndAttached.
     */
    virtual const double getCurrentSpringLength() const;

    /**
     * Return the location of the free end of the spring.
     * This is used, in particular, for rendering.
     * See tgBulletRenderer.
     */
    virtual const btVector3 getSpringEndpoint() const;
    
    /**
     * Returns the force currently in the spring, either compression only 
     * (e.g., a positive force only if isFreeEndAttached is false), 
     * or potentially either + or - force (if isFreeEndAttached == true).
     */
    virtual const double getSpringForce() const;

    /**
     * Return the direction of applied force for this spring
     */
    virtual const btVector3 * getDirection() const
    {
        return m_direction;
    }
    
protected:

    /**
     * Direction of the force that this spring will apply
     * A constant pointer to a constant direction.
     */
    btVector3 * m_direction;
    
    /**
     * Calculates the current forces that need to be applied to 
     * the rigid bodies, and applies them to the bodies of anchor1 and 
     * anchor2
     * Need to re-declare it here so it can be redefined in this child class.
     */
    virtual void calculateAndApplyForce(double dt);

private: 
    /** Ensures integrity of member variables */
    bool invariant(void) const;
};

#endif  // SRC_CORE_TG_BULLET_UNIDIR_COMPR_SPR_H_
