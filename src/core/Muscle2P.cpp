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
 * @file Muscle2P.cpp
 * @brief Definitions of members of classes Muscle2P and MuscleAnchor
 * @todo Split so only one class is defined per file.
 * $Id$
 */

// This module
#include "Muscle2P.h"
#include "muscleAnchor.h"
// The BulletPhysics library
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include <iostream>

Muscle2P::Muscle2P(btRigidBody * body1,
           btVector3 pos1,
           btRigidBody * body2,
           btVector3 pos2,
           double coefK,
           double dampingCoefficient) :
m_velocity(0.0),
m_damping(0.0),
m_coefK (coefK),
m_dampingCoefficient(dampingCoefficient),
anchor1(new muscleAnchor(body1, pos1, 0.0)),
anchor2(new muscleAnchor(body2, pos2, (pos2 - pos1).length()))
{
    m_restLength = pos1.distance(pos2);
    m_prevLength = m_restLength;
    
    // In case getActualLength() gets called before updateAnchorList
    m_anchors.insert(m_anchors.begin(), anchor1);
	m_anchors.insert(m_anchors.end(), anchor2);
}

btVector3 Muscle2P::calculateAndApplyForce(double dt)
{
    btVector3 force(0.0, 0.0, 0.0);
    double magnitude = 0.0;
    const btVector3 dist =
      anchor2->getWorldPosition() - anchor1->getWorldPosition();
      
    // These computations should occur for history regardless of motion
    const double currLength = dist.length();
    const btVector3 unitVector = dist / currLength;
    const double stretch = currLength - m_restLength;
    
    magnitude =  m_coefK * stretch;
    
    const double deltaStretch = currLength - m_prevLength;
    m_velocity = deltaStretch / dt;
    
    m_damping =  m_dampingCoefficient * m_velocity;
    
    if (abs(magnitude) * 1.0 < abs(m_damping))
    {
        m_damping =
          (m_damping > 0.0 ? magnitude * 1.0 : -magnitude * 1.0);
    }
    
    magnitude += m_damping;
    
    #if (0)
    std::cout << "Length: " << dist.length() << " rl: " << m_restLength <<std::endl; 
    #endif
      
    if (dist.length() > m_restLength)
    {   
        force = unitVector * magnitude; 
    }
    else
    {
        // Leave force as the zero vector
    }
    
    // Finished calculating, so can store things
    m_prevLength = currLength;

    //Now Apply it to the connected two bodies
    btVector3 point1 = this->anchor1->getRelativePosition();
    this->anchor1->attachedBody->activate();
    this->anchor1->attachedBody->applyImpulse(force*dt,point1);

    btVector3 point2 = this->anchor2->getRelativePosition();
    this->anchor2->attachedBody->activate();
    this->anchor2->attachedBody->applyImpulse(-force*dt,point2);

    return force;
}

void Muscle2P::setRestLength( const double newRestLength)
{
    // Assume we've already put this through a motor model
    // But check anyway
    assert(newRestLength > 0.0);
    
    m_restLength = newRestLength;
}

const double Muscle2P::getRestLength() const
{
    return m_restLength;
}

const btScalar Muscle2P::getActualLength() const
{
    const btVector3 dist =
      this->anchor2->getWorldPosition() - this->anchor1->getWorldPosition();
    return dist.length();
}

const double Muscle2P::getTension() const
{
    double tension = (getActualLength() - m_restLength) * m_coefK;
    tension = (tension < 0.0) ? 0.0 : tension;
    return tension;
}

Muscle2P::~Muscle2P()
{
    #if (0)
    std::cout << "Destroying Muscle2P" << std::endl;
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
