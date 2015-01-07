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
 * @file tgBasicActuator.cpp
 * @brief Contains the definitions of members of class tgBasicActuator
 * @author Brian Tietz
 * $Id$
 */

// This Module
#include "tgBulletSpringCable.h"
#include "tgBasicActuator.h"
#include "tgModelVisitor.h"
#include "tgWorld.h"
// The Bullet Physics Library
#include "LinearMath/btQuickprof.h"

// The C++ Standard Library
#include <cmath>
#include <deque> // For history
#include <iostream>
#include <stdexcept>

using namespace std;

void tgBasicActuator::constructorAux()
{
  // Precondition
    assert(m_pHistory != NULL);
    prevVel = 0.0;
    if (m_springCable == NULL)
    {
        throw std::invalid_argument("Pointer to tgBulletSpringCable is NULL.");
    }
    else if (m_config.targetVelocity < 0.0)
    {
        throw std::invalid_argument("Target velocity is negative.");
    }
    else if (m_config.minActualLength < 0.0)
    {
        throw std::invalid_argument("Minimum length is negative.");
    }
    else
    {
        logHistory();
    }
}
tgBasicActuator::tgBasicActuator(tgBulletSpringCable* muscle,
                   const tgTags& tags,
                   tgSpringCableActuator::Config& config) :
    tgSpringCableActuator(muscle, tags, config),
    m_preferredLength(m_restLength)
{
    constructorAux();

    // Postcondition
    assert(invariant());
    assert(m_preferredLength == m_restLength);
}

tgBasicActuator::~tgBasicActuator()
{
    //std::cout << "deleting linear string" << std::endl;
    // Should have already torn down.
}
    
void tgBasicActuator::setup(tgWorld& world)
{
    // This needs to be called here in case the controller needs to cast
    notifySetup();
    tgModel::setup(world);
}

void tgBasicActuator::teardown()
{
    // Do not notify teardown. The controller has already been deleted.
    tgModel::teardown();
}
    
void tgBasicActuator::step(double dt) 
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgBasicActuator::step");
#endif //BT_NO_PROFILE   	
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive.");
    }
    else
    {   
        // Want to update any controls before applying forces
        notifyStep(dt); 
        m_springCable->step(dt);
        logHistory();  
        tgModel::step(dt);
    }
}

void tgBasicActuator::onVisit(const tgModelVisitor& r) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgBasicActuator::onVisit");
#endif //BT_NO_PROFILE	
    r.render(*this);
}
    
void tgBasicActuator::logHistory()
{
    m_prevVelocity = m_springCable->getVelocity();

    if (m_config.hist)
    {
        m_pHistory->lastLengths.push_back(m_springCable->getActualLength());
        m_pHistory->lastVelocities.push_back(m_springCable->getVelocity());
        m_pHistory->dampingHistory.push_back(m_springCable->getDamping());
        m_pHistory->restLengths.push_back(m_springCable->getRestLength());
        m_pHistory->tensionHistory.push_back(m_springCable->getTension());
    }
}

void tgBasicActuator::setControlInput(double input)
{
    if (input < 0.0)
    {
      throw std::invalid_argument("Rest length is negative.");
    }
    else
    {
        m_preferredLength = input;
    }
}    

void tgBasicActuator::setControlInput(double input, double dt)
{
    if (input < 0.0)
    {
      throw std::invalid_argument("Rest length is negative.");
    }
    else
    {
        m_preferredLength = input;
        
        // moveMotors can change m_preferred length, so this goes here for now
        assert(m_preferredLength == input);
              
        moveMotors(dt);
    }

    // Postcondition
    assert(invariant());
    
}

void tgBasicActuator::moveMotors(double dt)
{
    // @todo add functions from muscle2P Bounded
    
    
    const double stiffness = m_springCable->getCoefK();
    // @todo: write invariant that checks this;
    assert(stiffness > 0.0);
    
    // Reverse the sign if restLength >= preferredLength
    // Velocity limiter
    double stepSize = m_config.targetVelocity * dt;
    const double actualLength = m_springCable->getActualLength(); 
    
    // First, change preferred length so we don't go over max tension
    if ((actualLength - m_preferredLength) * stiffness
            > m_config.maxTens)
    {
        m_preferredLength = actualLength - m_config.maxTens / stiffness;
    }
    
    double diff =  m_preferredLength - m_restLength;
    const double fabsDiff = abs(diff);
    
    /*
     * actualLength must be greater than minActualLength to shorten
     * diff > 0 means can always lengthen.
     */
    if ((actualLength > m_config.minActualLength) || 
    (diff > 0))
    {
        if (abs(diff) > stepSize)
        {      
          m_restLength += (diff/fabsDiff)*stepSize;
        }
        else
        {
            m_restLength += diff;
        }
    }
    
     m_restLength =
      (m_restLength > m_config.minRestLength) ? m_restLength : m_config.minRestLength;
     #if (0)
     std::cout << "RL: " << m_restLength << " M2P RL: " << m_springCable->getRestLength() << std::endl;
     
     
     std::cout  << "RL: " << m_restLength
     << " Vel: " << (m_restLength  -m_springCable->getRestLength()) / dt 
     << " prev Vel: " << prevVel
     << " force " << (actualLength - m_restLength)*stiffness << std::endl;
     prevVel = (m_restLength  -m_springCable->getRestLength()) / dt ;
     #endif
     m_springCable->setRestLength(m_restLength);
    
}

bool tgBasicActuator::invariant() const
{
    return
      (m_springCable != NULL) &&
      (m_pHistory != NULL) && 
      (m_config.targetVelocity >= 0.0) &&
      (m_config.minActualLength >= 0.0) &&
      (m_preferredLength >= 0.0);
}
