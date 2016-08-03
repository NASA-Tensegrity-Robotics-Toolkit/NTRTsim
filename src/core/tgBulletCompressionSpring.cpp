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
 * @author Drew Sabelhaus, Brian Mirletz, et al.
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
                double coefK,
                double coefD,
                double restLength) :
m_dampingForce(0.0),
m_velocity(0.0),
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
	throw std::invalid_argument("Rest length for a compression spring must be postiive.");
    }
	
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
      throw std::runtime_error("Compression spring has negative length, simulation stopping. Increase your stiffness coefficient.");
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
 * Returns the current force in the spring.
 * This is zero if the distance between anchors is larger than restLength.
 */
const double tgBulletCompressionSpring::getSpringForce() const
{
    double springForce = 0.0;
    if( getCurrentAnchorDistance() >= m_restLength)
    {
        std::cout << "Zero force on compression spring" << std::endl;
	springForce = 0.0;
    }
    else
    {
        springForce = (m_restLength - getCurrentAnchorDistance()) * m_coefK;
    }
    // confirm: springForce should never be negative.
    assert(springForce >= 0.0);
    
    return springForce;
}

/**
 * Returns the current length of the spring, either restLength or less.
 */
const double tgBulletCompressionSpring::getCurrentSpringLength() const
{
    double springLength = m_restLength;
    if( getCurrentAnchorDistance() >= m_restLength)
    {
        // if the distance between the two anchors is larger than the rest length of the spring,
        // it means (intuitively) that one end of the spring is not touching a rigid body.
        std::cout << "Compression spring is unloaded" << std::endl;
	springLength = m_restLength;
    }
    else
    {
        springLength = (m_restLength - getCurrentAnchorDistance());
    }
    
    return springLength;
}

/**
 * This is the method that does the heavy lifting in this class.
 * Called by step, it calculates what force the spring is experiencing,
 * and applies that force to the rigid bodies it connects to.
 */
void tgBulletCompressionSpring::calculateAndApplyForce(double dt)
{
    // Apply a force only if the spring is loaded.
    if( getCurrentAnchorDistance() < m_restLength)
    {
        // Create variables to hold the results of these computations
        btVector3 force(0.0, 0.0, 0.0);
        double magnitude = 0.0;
      
        // Calculate the distance between the anchors
        // Here, 'stretch' is the delta_x in F = k delta_x.
        const btVector3 dist =
	  anchor2->getWorldPosition() - anchor1->getWorldPosition();
	const double currLength = dist.length();
	const double stretch = currLength - m_restLength;
    
	// The unit vector of the direction of the force will be needed later
	const btVector3 unitVector = dist / currLength;    

	// The magnitude of the force due to the spring is simple to calculate:
	magnitude =  m_coefK * stretch;

	// The magnitude of the force due to damping is a bit more complicated.
	// Take an approximated derivative to estimate the velocity of the
	// tip of the compression spring:
	const double deltaStretch = currLength - m_prevLength;
	m_velocity = deltaStretch / dt;

	// The damping force for this timestep
	m_dampingForce = m_coefD * m_velocity;

	// Adjust the forces if needed...?
	// copied from tgBulletSpringCable. Not quite sure why this check
	// is done. Why do we want to flip the direction of the spring force
	// if the damping force is greater than the spring force?
	if (abs(magnitude) * 1.0 < abs(m_dampingForce))
	{
	  m_dampingForce =
          (m_dampingForce > 0.0 ? magnitude * 1.0 : -magnitude * 1.0);
	}

	// Add the damping force to the force from the spring.
	magnitude += m_dampingForce;

	// Debugging
        #if (0)
	std::cout << "Length: " << dist.length() << " rl: " << m_restLength <<std::endl; 
        #endif
      
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
}

const std::vector<const tgSpringCableAnchor*> tgBulletCompressionSpring::getAnchors() const
{
    return tgCast::constFilter<tgBulletSpringCableAnchor, const tgSpringCableAnchor>(m_anchors);
}

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
