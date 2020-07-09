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
 * @file tgBulletUnidirComprSpr.cpp
 * @brief Definitions of members of class tgBulletUnidirComprSpr
 * @author Drew Sabelhaus, et al.
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// This module
#include "tgBulletUnidirComprSpr.h"
#include "tgBulletSpringCableAnchor.h"
#include "tgCast.h"
// The BulletPhysics library
#include "BulletDynamics/Dynamics/btRigidBody.h"
// The C++ standard library
#include <iostream>
#include <stdexcept>

/**
 * The main constructor for this class
 * In comparison to the tgSpringCable vs. tgBulletSpringCable class inheritance,
 * there is no need for the tgCast here, since the same type of anchor is used
 * in both tgBulletCompressionSpring and tgBulletUnidirComprSpr.
 */
tgBulletUnidirComprSpr::tgBulletUnidirComprSpr(
		const std::vector<tgBulletSpringCableAnchor*>& anchors,
		bool isFreeEndAttached,
                double coefK,
                double coefD,
                double restLength,
		btVector3 * direction) :
tgBulletCompressionSpring(anchors, isFreeEndAttached, coefK, coefD, restLength),
m_direction(direction)
{
    // Since tgBulletCompressionSpring takes care of everything else,
    // just need to check that direction is valid, and then check the invariant.

    assert(invariant());

    #if (0)
    std::cout << "tgBulletUnidirComprSpr constructor, ";
    std::cout << "direction is: ";
    std::cout << "(" << m_direction->x() << ",";
    std::cout << m_direction->y() << ",";
    std::cout << m_direction->z() << ")" << std::endl;
    #endif

    // Check that m_direction is indeed a unit vector.
    // A dot product with (1,1,1) should always return the scalar 1 if
    // the vector is indeed a unit vector.
    // @TODO: make this work with arbitrary unit vectors. Those dot products
    // are probably not exactly 1.0, but 1.0somethingsmall. 
    double dotproduct = m_direction->dot( btVector3(1,1,1) );
    // @TODO: Note that this should also check if the length at start is negative:
    // this would indicate that the dotproduct must be negative.
    if( fabs(dotproduct) != 1.0 ){
      std::cout << "Error: m_direction is not a unit vector. Its dot product" <<
	" with (1,1,1) is " << dotproduct << std::endl;
      throw std::invalid_argument("Direction must be a unit vector.");
    }
}

// Destructor has to the responsibility of destroying the anchors also,
// as well as the btVector3 direction.
tgBulletUnidirComprSpr::~tgBulletUnidirComprSpr()
{
    #if (0)
    std::cout << "Destroying tgBulletUnidirComprSpr..." << std::endl;
    #endif
}

// The step function is what's called from other places in NTRT.
void tgBulletUnidirComprSpr::step(double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive!");
    }

    calculateAndApplyForce(dt);

    // If the spring distance has gone negative, output a scary warning.
    // TO-DO: find a way to apply a hard stop here instead.
    if( getCurrentSpringLength() < 0.0)
    {
      std::cout << "WARNING! UNIDIRECTIONAL COMPRESSION SPRING IS "
		<< "LESS THAN ZERO LENGTH. YOUR SIMULATION MAY BE INACCURATE FOR "
		<< "ANY TIMESTEPS WHEN THIS MESSAGE APPEARS. " << std::endl;
      std::cout << "Current spring length is " << getCurrentSpringLength()
		<< std::endl << std::endl;

      /* If we wanted the simulator to completely quit instead:
      std::cout << "Error, unidirectional compression spring length "
		<< "is negative. Length is: " << getCurrentSpringLength()
       		<< std::endl;
      throw std::runtime_error("Unidirectional compression spring has negative length, simulation stopping. Increase your stiffness coefficient.");
      */
    }
    
    assert(invariant());
}

/**
 * Dot getCurrentAnchorDistance with m_direction.
 */
const double tgBulletUnidirComprSpr::getCurrentAnchorDistanceAlongDirection() const
{
  // btVector3 between the two anchors
  const btVector3 dist =
    anchor2->getWorldPosition() - anchor1->getWorldPosition();

  // Dot it with the direction of this spring, should return a double.
  // m_direction is a pointer, so dereference first.
  double currAnchDistAlongDir = dist.dot( (*getDirection()) );
  return currAnchDistAlongDir;
}

/**
 * Returns the current length of the spring. If isFreeEndAttached,
 * this can be either greater or less than m_restLength. If not, then spring
 * can only exist in compression (less than m_restLength).
 * This only calculates the distance in the stated direction.
 */
const double tgBulletUnidirComprSpr::getCurrentSpringLength() const
{
    // initialize to the default value.
    // if the distance between the two anchors is larger
    // than the rest length of the spring, it means (intuitively) that
    // one end of the spring is not touching a rigid body.
    double springLength = getRestLength();

    // store this variable so we don't have multiple calls to the same function.
    //double currAnchorDist = getCurrentAnchorDistance();
    double currAnchorDistAlongDir = getCurrentAnchorDistanceAlongDirection();

    if( isFreeEndAttached() )
    {
      // The spring length is always the distance between anchors, projected along
      // the direction vector.
      springLength = currAnchorDistAlongDir;
    }
    else if( currAnchorDistAlongDir < getRestLength() )
    {
      // the spring is not attached at its free end, but is in compression,
      // so there is a length change.
      springLength = currAnchorDistAlongDir;
    }
    
    return springLength;
}

/**
 * Returns the location of the endpoint of the spring in space. 
 * The renderer uses this to draw lines more easily.
 */
const btVector3 tgBulletUnidirComprSpr::getSpringEndpoint() const
{
  // The spring endpoint will always be the sum of the beginning point
  // and the (spring length times the vector in the direction of the spring).
  // Note that, by this point, it should have been checked that m_direction,
  // which is what's returned by getDirection, is indeed a unit vector.
  // Direction is a pointer, so it must be dereferenced first.
  // Direction has the opposite magnitude of what unitVector woudld be in the parent
  // class, so this is a "+" instead of a "-".
  return anchor1->getWorldPosition() + 
    getCurrentSpringLength() * (*getDirection());
}

/**
 * Returns the current force in the spring.
 * If ~isFreeEndAttached, 
 * this is zero if the distance between anchors is larger than restLength.
 * Note that this does NOT include the force due to the damper, since that force
 * is only temporary and isn't reallyt "the force in the spring" per se.
 */
const double tgBulletUnidirComprSpr::getSpringForce() const
{
    // Since the getCurrentSpringLength function already includes the check
    // against m_isFreeEndAttached, as well as a projection along m_direction,
    double springForce = - getCoefK() * (getCurrentSpringLength() - getRestLength());

    //DEBUGGING
    #if (0)
    std::cout << "getCoefK: " << getCoefK() << " getCurrentSpringLength(): "
	      << getCurrentSpringLength() << " getRestLength: "
	      << getRestLength() <<std::endl;
    #endif

    //DEBUGGING
    if (0) {
      std::cout << "Called getSpringForce "
		<< "inside tgBulletUnidirComprSpr" << std::endl;
    }
    
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
void tgBulletUnidirComprSpr::calculateAndApplyForce(double dt)
{

    // Create variables to hold the results of these computations
    btVector3 force(0.0, 0.0, 0.0);
    // the following ONLY includes forces due to K, not due to damping.
    double magnitude = getSpringForce();

    // hold this variable so we don't have to call the accessor function twice.
    const double currLength = getCurrentSpringLength();

    // Get the unit vector for the direction of the force.
    // This spring applies a force only along m_direction.
    // again, it should have been checked that m_direction is a unit vector by now.
    // Direction is a pointer, so must be dereferenced first.
    // TO-DO: justify the "-" here. This works, not sure exactly why, but probably
    //    has to do with choice of which anchor to subtract from the other.
    const btVector3 unitVector = - (*getDirection());

    // Calculate the damping force for this timestep.
    // Take an approximated derivative to estimate the velocity of the
    // tip of the compression spring:
    const double changeInDeltaX = currLength - m_prevLength;
    m_velocity = changeInDeltaX / dt;

    // The damping force for this timestep
    // Like with spring force, a positive velocity should result in a
    // force acting against the movement of the tip of the spring.
    m_dampingForce = - getCoefD() * m_velocity;

    // Debugging
    #if (0)
      std::cout << "tgBulletUnidirComprSpr::calculateAndApplyForce  " << std::endl;
      std::cout << "Length: " << getCurrentSpringLength() << " rl: " << getRestLength() <<std::endl;
      std::cout << "SpringForce: " << magnitude << " DampingForce: " << m_dampingForce <<std::endl;
    #endif

    // Add the damping force to the force from the spring.
    magnitude += m_dampingForce;
    
    // Project the force into the direction of the line between the two anchors
    force = unitVector * magnitude; 
    
    // Store the calculated length as the previous length
    m_prevLength = currLength;
    
    //Now Apply it to the connected two bodies.
    //Note that the point of applied force on the second body is NOT the same
    // as the location specified by getSpringEndpoint. The anchor is the point
    // to apply the force, and getSpringEndpoint is only used for rendering
    // purposes: it makes more sense to have the spring free end "floating in space"
    // in the rendering.
    btVector3 point1 = this->anchor1->getRelativePosition();
    this->anchor1->attachedBody->activate();
    this->anchor1->attachedBody->applyImpulse(force*dt,point1);
    
    btVector3 point2 = this->anchor2->getRelativePosition();
    this->anchor2->attachedBody->activate();
    this->anchor2->attachedBody->applyImpulse(-force*dt,point2);
}

bool tgBulletUnidirComprSpr::invariant(void) const
{
  // Instead of checking for less than zero length, just output a warning.
  // @TODO: find some way of dealing with Bullet's less-than-zero-length between
  // rigid bodies that are colliding.
  if( (m_prevLength < 0) ) {
    std::cout << "WARNING! UNIDIRECTIONAL COMPRESSION SPRING IS "
	      << "LESS THAN ZERO LENGTH. YOUR SIMULATION MAY BE INACCURATE FOR "
	      << "ANY TIMESTEPS WHEN THIS MESSAGE APPEARS. " << std::endl;
  }
  // Used to have m_prevLength >= 0.0 && 
  return (m_coefK > 0.0 &&
	  m_coefD >= 0.0 &&
	  m_restLength >= 0.0 &&
	  anchor1 != NULL &&
	  anchor2 != NULL &&
	  m_direction != NULL &&
	  m_anchors.size() >= 2);
}
