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
 * @file tgLinearString.cpp
 * @brief Contains the definitions of members of class tgLinearString
 * @author Brian Tietz
 * $Id$
 */

// This Module
#include "Muscle2P.h"
#include "tgLinearString.h"
#include "tgModelVisitor.h"
#include "tgWorld.h"
// The Bullet Physics Library
#include "LinearMath/btQuickprof.h"

// The C++ Standard Library
#include <cmath>
#include <deque> // For history
#include <iostream>
#include <stdexcept>

#define ACCELERATION_CAP

using namespace std;

void tgLinearString::constructorAux()
{
  // Precondition
    assert(m_pHistory != NULL);
    prevVel = 0.0;
    if (m_muscle == NULL)
    {
        throw std::invalid_argument("Pointer to Muscle2P is NULL.");
    }
    else if (m_config.targetVelocity < 0.0)
    {
        throw std::invalid_argument("Target velocity is negative.");
    }
    else if (m_config.maxAcc < 0.0)
    {
        throw std::invalid_argument("Maximum acceleration is negative.");
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
tgLinearString::tgLinearString(Muscle2P* muscle,
                   const tgTags& tags,
                   tgBaseString::Config& config) :
    m_muscle(muscle),
    tgBaseString(tags, config, muscle->getRestLength(), muscle->getActualLength())
{
    constructorAux();

    // Postcondition
    assert(invariant());
    assert(m_muscle == muscle);
    assert(m_preferredLength == m_restLength);
}

tgLinearString::~tgLinearString()
{
    //std::cout << "deleting linear string" << std::endl;
    // Should have already torn down.
    delete m_muscle;
}
    
void tgLinearString::setup(tgWorld& world)
{
    notifySetup();
    tgModel::setup(world);
}

void tgLinearString::teardown()
{
    // Do not notify teardown. The controller has already been deleted.
    tgModel::teardown();
}
    
void tgLinearString::step(double dt) 
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgLinearString::step");
#endif //BT_NO_PROFILE   	
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive.");
    }
    else
    {   
        // Want to update any controls before applying forces
        notifyStep(dt); 
        m_muscle->calculateAndApplyForce(dt);
        logHistory();  
        tgModel::step(dt);
    }
}

void tgLinearString::onVisit(const tgModelVisitor& r) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgLinearString::onVisit");
#endif //BT_NO_PROFILE	
    r.render(*this);
}
    
void tgLinearString::logHistory()
{
    m_prevVelocity = m_muscle->getVelocity();

    if (m_config.hist)
    {
        m_pHistory->lastLengths.push_back(m_muscle->getActualLength());
        m_pHistory->lastVelocities.push_back(m_muscle->getVelocity());
        m_pHistory->dampingHistory.push_back(m_muscle->getDamping());
        m_pHistory->restLengths.push_back(m_muscle->getRestLength());
        m_pHistory->tensionHistory.push_back(m_muscle->getTension());
    }
}
    
const double tgLinearString::getStartLength() const
{
    return m_startLength;
}
    
const double tgLinearString::getCurrentLength() const
{
    return m_muscle->getActualLength();
}  

const double tgLinearString::getTension() const
{
    return m_muscle->getTension();
}
    
const double tgLinearString::getRestLength() const
{
    return m_muscle->getRestLength();
}

const double tgLinearString::getVelocity() const
{
    return m_muscle->getVelocity();
}

void tgLinearString::setRestLength(double newLength, float dt)
{
    if (newLength < 0.0)
    {
      throw std::invalid_argument("Rest length is negative.");
    }
    else
    {
        m_preferredLength = newLength;
        
        // moveMotors can change m_preferred length, so this goes here for now
        assert(m_preferredLength == newLength);
              
	moveMotors(dt);
    }

    // Postcondition
    assert(invariant());
    
}

void tgLinearString::setPrefLength(double newLength)
{
    if (newLength < 0.0)
    {
      throw std::invalid_argument("Rest length is negative.");
    }
    else
    {
        m_preferredLength = newLength;
    }
}

void tgLinearString::moveMotors(double dt)
{
    // @todo add functions from muscle2P Bounded
    
    
    const double stiffness = m_muscle->getCoefK();
    // @todo: write invariant that checks this;
    assert(stiffness > 0.0);
    
    // Reverse the sign if restLength >= preferredLength
    // Velocity limiter
    double stepSize = m_config.targetVelocity * dt;
    // Acceleration limiter
    const double velChange = m_config.maxAcc * dt;
    const double actualLength = m_muscle->getActualLength();
    const double mostRecentVelocity = m_prevVelocity;
    
    
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
        //Cap Velocity
#ifdef ACCELERATION_CAP
      if (abs((diff/fabsDiff) * m_config.targetVelocity -
          mostRecentVelocity) >
          velChange)
      {
          // Cap Acceleration
          stepSize = velChange * dt;
      }
#endif      
      m_restLength += (diff/fabsDiff)*stepSize;
    }
    else
    {
#ifdef ACCELERATION_CAP
        if (abs(diff/dt - mostRecentVelocity) > velChange)
        {
            // Cap Acceleration
            if (diff != 0) 
            {
              diff = (diff/fabsDiff) * velChange * dt;
            }
            else
            { 
                // If m_prevVelocity was zero, it would be smaller than
                // velChange. Therefore preVelocity is valid for 
                // figuring out direction
              diff = -(mostRecentVelocity / abs(mostRecentVelocity)) *
                     velChange * dt;
            }
        }
#endif
        m_restLength += diff;
    }
    }
    
     m_restLength =
      (m_restLength > m_config.minRestLength) ? m_restLength : m_config.minRestLength;
     #if (0)
     std::cout << "RL: " << m_restLength << " M2P RL: " << m_muscle->getRestLength() << std::endl;
     
     
     std::cout  << "RL: " << m_restLength
     << " Vel: " << (m_restLength  -m_muscle->getRestLength()) / dt 
     << " prev Vel: " << prevVel
     << " force " << (actualLength - m_restLength)*stiffness << std::endl;
     prevVel = (m_restLength  -m_muscle->getRestLength()) / dt ;
     #endif
     m_muscle->setRestLength(m_restLength);
    
}

/* A modified version of setRestLength that performs said operation
 * in a single call, as opposed to the original method, which requires
 * multiple calls, since it relies on moveMotors.
 * @param newLength the new rest length of the string.
 */
void tgLinearString::setRestLengthSingleStep(double newLength)
{
    if (newLength < 0.0)
    {
      throw std::invalid_argument("Rest length is negative.");
    }
    else
    {
        m_preferredLength = newLength;
        
        // moveMotors can change m_preferred length, so this goes here for now
        assert(m_preferredLength == newLength);

	// we should assert something to confirm consistency since we're
	// not calling moveMotors anymore. Does anything else need to
	// change when restLength is changed? -Drew 7-1-14

	m_muscle->setRestLength(newLength);
	m_restLength = newLength;
	m_preferredLength = newLength;
    }

    // Postcondition
    assert(invariant());
    
}

void tgLinearString::tensionMinLengthController(const double targetTension,
                      float dt)
{

    const double stiffness = m_muscle->getCoefK();
    // @todo: write invariant that checks this;
    assert(stiffness > 0.0);
    
    const double currentTension = m_muscle->getTension();
    const double delta = targetTension - currentTension;
    double diff = delta/stiffness; 
    const double currentLength = m_muscle->getRestLength();
    
    const double newLength = m_restLength - diff;

    m_preferredLength =
      (newLength > m_config.minRestLength) ? newLength : m_config.minRestLength;
#if (0)
    std::cout << "m_preferred: " << m_preferredLength << std::endl;
#endif
    moveMotors(dt);
}

const tgBaseString::BaseStringHistory& tgLinearString::getHistory() const
{
    return *m_pHistory;
}

bool tgLinearString::invariant() const
{
    return
      (m_muscle != NULL) &&
      (m_pHistory != NULL) && 
      (m_config.targetVelocity >= 0.0) &&
      (m_config.maxAcc >= 0.0) &&
      (m_config.minActualLength >= 0.0) &&
      (m_preferredLength >= 0.0);
}
