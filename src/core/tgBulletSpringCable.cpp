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
 * @file tgBulletSpringCable.cpp
 * @brief Definitions of members of classes tgBulletSpringCable
 * $Id$
 */

// This module
#include "tgBulletSpringCable.h"
#include "tgBulletSpringCableAnchor.h"
#include "tgCast.h"
// The BulletPhysics library
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include <iostream>
#include <stdexcept>

tgBulletSpringCable::tgBulletSpringCable( const std::vector<tgBulletSpringCableAnchor*>& anchors,
                double coefK,
                double dampingCoefficient,
                double pretension) :
tgSpringCable(tgCast::filter<tgBulletSpringCableAnchor, tgSpringCableAnchor>(anchors),
                coefK, dampingCoefficient, pretension),
m_anchors(anchors),
anchor1(anchors.front()),
anchor2(anchors.back())
{
    assert(m_anchors.size() >= 2);
    assert(invariant());
    // tgSpringCable does heavy lifting as far as determining rest length
}


tgBulletSpringCable::~tgBulletSpringCable()
{
    #if (0)
    std::cout << "Destroying tgBulletSpringCable" << std::endl;
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

void tgBulletSpringCable::step(double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive!");
    }

    calculateAndApplyForce(dt);
    assert(invariant());
}

void tgBulletSpringCable::calculateAndApplyForce(double dt)
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
}

const double tgBulletSpringCable::getActualLength() const
{
    const btVector3 dist =
      this->anchor2->getWorldPosition() - this->anchor1->getWorldPosition();
    return dist.length();
}

const double tgBulletSpringCable::getTension() const
{
    double tension = (getActualLength() - m_restLength) * m_coefK;
    tension = (tension < 0.0) ? 0.0 : tension;
    return tension;
}

const std::vector<const tgSpringCableAnchor*> tgBulletSpringCable::getAnchors() const
{
    return tgCast::constFilter<tgBulletSpringCableAnchor, const tgSpringCableAnchor>(m_anchors);
}

bool tgBulletSpringCable::invariant(void) const
{
    return (m_coefK > 0.0 &&
    m_dampingCoefficient >= 0.0 &&
    m_prevLength >= 0.0 &&
    m_restLength >= 0.0 &&
    anchor1 != NULL &&
    anchor2 != NULL &&
    m_anchors.size() >= 2);
}
