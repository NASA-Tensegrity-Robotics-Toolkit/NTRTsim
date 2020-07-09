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

/**
 * @file tgBulletCompressionSpring.cpp
 * @brief Definitions of members of class tgBulletCompressionSpring
 * @author Drew Sabelhaus, et al.
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// This module
#include "tgBulletCompressionSpring.h"
#include "tgBulletSpringCableAnchor.h"
#include "tgCast.h"
// The BulletPhysics library
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include <iostream>
#include <stdexcept>

/**
 * The main constructor for this class
 */
tgBulletCompressionSpring::tgBulletCompressionSpring( const std::vector<tgBulletSpringCableAnchor*>& anchors,
		bool isFreeEndAttached,
                double coefK,
                double coefD,
                double restLength) :
m_dampingForce(0.0),
m_velocity(0.0),
m_isFreeEndAttached(isFreeEndAttached),
m_coefK(coefK),
m_coefD(coefD),
m_restLength(restLength),
m_anchors(anchors),
anchor1(anchors.front()),
anchor2(anchors.back())
{
    // There should be two anchors for a compression spring.
    assert(m_anchors.size() == 2);

    // Constructor from tgSpringCable:
    assert(coefK > 0.0);
    assert(coefD >= 0.0);

    // the rest length of the spring has to be positive
    if (m_restLength <= 0.0)
    {
	throw std::invalid_argument("Rest length for a compression spring must be postive.");
    }

    // Debugging
    #if (0)
    btVector3 anchor1pos = anchor1->getWorldPosition();
    btVector3 anchor2pos = anchor2->getWorldPosition();
    std::cout << "Location of the starting and ending point of the two anchors:" << std::endl;
    std::cout << "Anchor1: (" << anchor1pos.x() << ", ";
    std::cout << anchor1pos.y() << ", ";
    std::cout << anchor1pos.z() << ")" << std::endl;
    std::cout << "Anchor2: (" << anchor2pos.x() << ", ";
    std::cout << anchor2pos.y() << ", ";
    std::cout << anchor2pos.z() << ")" << std::endl;
    #endif
	
    m_prevLength = m_restLength;
    
    assert(invariant());
}

// Destructor has to the responsibility of destroying the anchors also.
tgBulletCompressionSpring::~tgBulletCompressionSpring()
{
    #if (0)
    std::cout << "Destroying tgBulletCompressionSpring" << std::endl;
    #endif
    
    std::size_t n = m_anchors.size();
    
    // Make absolutely sure these are deleted, in case we have a poorly timed reset
    if (m_anchors[0] != anchor1)
    {
        delete anchor1;
    }
    
    if (m_anchors[n-1] != anchor2)
    {
        delete anchor2;
    }
    
    for (std::size_t i = 0; i < n; i++)
    {
        delete m_anchors[i];
    }

    
    m_anchors.clear();
}

// The step function is what's called from other places in NTRT.
void tgBulletCompressionSpring::step(double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive!");
    }

    calculateAndApplyForce(dt);

    // If the spring distance has gone negative, crash the simulator on purpose.
    // TO-DO: find a way to apply a hard stop here instead.
    if( getCurrentSpringLength() <= 0.0)
    {
      throw std::runtime_error("Compression spring has negative length, simulation stopping. Increase your stiffness coefficient. TO-DO: implement a 'hard stop' inside the step method of tgBulletCompressionSpring instead of crashing the simulator.");
    }
    
    assert(invariant());
}

/**
 * Returns the distance between the two anchors.
 * This is similar to the getActualLength function in tgBulletSpringCable.
 */
const double tgBulletCompressionSpring::getCurrentAnchorDistance() const
{
    const btVector3 dist =
      this->anchor2->getWorldPosition() - this->anchor1->getWorldPosition();
    return dist.length();
}

/**
 * Returns the current length of the spring. If isFreeEndAttached,
 * this can be either greater or less than m_restLength. If not, then spring
 * can only exist in compression (less than m_restLength).
 */
const double tgBulletCompressionSpring::getCurrentSpringLength() const
{
    // initialize to the default value.
    // If not attached, if the distance between the two anchors is larger
    // than the rest length of the spring, it means (intuitively) that
    // one end of the spring is not touching a rigid body.
    double springLength = getRestLength();

    // store this variable so we don't have multiple calls to the same function.
    double currAnchorDist = getCurrentAnchorDistance();

    if( isFreeEndAttached() )
    {
      // The spring length is always the distance between anchors.
      springLength = currAnchorDist;
    }
    else if( currAnchorDist < getRestLength() )
    {
      // the spring is not attached at its free end, but is in compression,
      // so there is a length change.
      springLength = currAnchorDist;
    }
    
    return springLength;
}


/**
 * Returns the unit vector in the direction of this spring.
 */
const btVector3 tgBulletCompressionSpring::getAnchorDirectionUnitVector() const
{
    // Get the unit vector for the direction of the force, this is needed for
    // applying the force to the rigid bodies.
    const btVector3 dist =
	  anchor2->getWorldPosition() - anchor1->getWorldPosition();
    // The unit vector of the direction of the force will be needed later
    // In order to have a positive force move the two rigid bodies away
    // from each other, this unit vector must be in the opposite direction
    // of this calculation. Otherwise, a positive force brings them closer
    // together. Needs a minus.
    // note that dist.length is a scalar double.
    const btVector3 unitVector = - dist / dist.length();
    return unitVector;
}

/**
 * Returns the location of the endpoint of the spring in space. 
 * The renderer uses this to draw lines more easily.
 */
const btVector3 tgBulletCompressionSpring::getSpringEndpoint() const
{
  // The spring endpoint will always be the sum of the beginning point
  // and the (spring length times the unit vector in the direction of the spring).
  // Unit vector is in opposite direction here.
  return anchor1->getWorldPosition() - 
    getCurrentSpringLength() * getAnchorDirectionUnitVector();
}

/**
 * Returns the current force in the spring.
 * If ~isFreeEndAttached, 
 * this is zero if the distance between anchors is larger than restLength.
 * Note that this does NOT include the force due to the damper, since that force
 * is only temporary and isn't reallyt "the force in the spring" per se.
 */
const double tgBulletCompressionSpring::getSpringForce() const
{
    // Since the getCurrentSpringLength function already includes the check
    // against m_isFreeEndAttached:
    double springForce = - getCoefK() * (getCurrentSpringLength() - getRestLength());

    // Debugging
    #if (0)
    std::cout << "getCoefK: " << getCoefK() << " getCurrentSpringLength(): "
	      << getCurrentSpringLength() << " getRestLength: "
	      << getRestLength() <<std::endl;
    #endif
    
    // A negative delta_X should result in a positive force.
    // note that if m_isFreeEndAttached == false, then springForce >= 0 always,
    // since then the calculation above will have a (restlength - restLength) term.
    // quick check to be sure of that:
    if( !isFreeEndAttached())
    {
        assert(springForce >= 0.0);
    }
    
    return springForce;
}

/**
 * This is the method that does the heavy lifting in this class.
 * Called by step, it calculates what force the spring is experiencing,
 * and applies that force to the rigid bodies it connects to.
 */
void tgBulletCompressionSpring::calculateAndApplyForce(double dt)
{
    // Create variables to hold the results of these computations
    btVector3 force(0.0, 0.0, 0.0);
    // the following ONLY includes forces due to K, not due to damping.
    double magnitude = getSpringForce();

    // hold these variables so we don't have to call the accessor function twice.
    const double currLength = getCurrentSpringLength();
    const btVector3 unitVector = getAnchorDirectionUnitVector();

    // Calculate the damping force for this timestep.
    // Take an approximated derivative to estimate the velocity of the
    // tip of the compression spring:
    // TO-DO: MAKE THIS A BETTER APPROXIMATION TO THE DERIVATIVE!!
    const double changeInDeltaX = currLength - m_prevLength;
    m_velocity = changeInDeltaX / dt;

    // The damping force for this timestep
    // Like with spring force, a positive velocity should result in a
    // force acting against the movement of the tip of the spring.
    m_dampingForce = - getCoefD() * m_velocity;

    // Debugging
    #if (0)
      std::cout << "Length: " << getCurrentSpringLength() << " rl: " << getRestLength() <<std::endl;
      std::cout << "SpringForce: " << magnitude << " DampingForce: " << m_dampingForce <<std::endl;
    #endif

    // Add the damping force to the force from the spring.
    magnitude += m_dampingForce;
    
    // Project the force into the direction of the line between the two anchors
    force = unitVector * magnitude; 
    
    // Store the calculated length as the previous length
    m_prevLength = currLength;
    
    //Now Apply it to the connected two bodies
    btVector3 point1 = this->anchor1->getRelativePosition();
    this->anchor1->attachedBody->activate();
    this->anchor1->attachedBody->applyImpulse(force*dt,point1);
    
    btVector3 point2 = this->anchor2->getRelativePosition();
    this->anchor2->attachedBody->activate();
    this->anchor2->attachedBody->applyImpulse(-force*dt,point2);
}

// returns the list of (two) anchors for this class.
const std::vector<const tgSpringCableAnchor*>tgBulletCompressionSpring::getAnchors() const
{
    return tgCast::constFilter<tgBulletSpringCableAnchor, const tgSpringCableAnchor>(m_anchors);
}

// The invariant, for checking that everything is OK.
bool tgBulletCompressionSpring::invariant(void) const
{
    return (m_coefK > 0.0 &&
    m_coefD >= 0.0 &&
    m_prevLength >= 0.0 &&
    m_restLength >= 0.0 &&
    anchor1 != NULL &&
    anchor2 != NULL &&
    m_anchors.size() >= 2);
}
