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

#include "tgCordeModel.h"

#include "core/tgModelVisitor.h"
#include "core/tgWorld.h"

#include "dev/Corde/CordeModel.h"
#include "dev/Corde/cordeCollisionObject.h"

#include <stdexcept>
#include <complex> // std::abs (just abs has issues below 1)

tgCordeModel::Config::Config(tgBaseString::Config motor_config,
								CordeModel::Config string_config) :
motorConfig(motor_config),
stringConfig(string_config)
{
	// Assertions should be handled by the respective configs.
}

tgCordeModel::tgCordeModel(cordeCollisionObject* string,
							tgBaseString::Config motor_config,
							const tgTags& tags) :
m_string(string),
tgBaseString(tags, motor_config, string->getRestLength(), string->getActualLength()),
m_prevLength(string->getRestLength())
{	

}
    
tgCordeModel::~tgCordeModel()
{
    
}
    
void tgCordeModel::setup(tgWorld& world)
{
    notifySetup();
    tgModel::setup(world);
}

void tgCordeModel::teardown()
{
    // World handles deleting collision object
    m_string = NULL;
    tgModel::teardown();
}

void tgCordeModel::step(const double dt)
{
	notifyStep(dt);
	tgModel::step(dt);
	logHistory(dt);
}

/**
* Call tgModelVisitor::render() on self and all descendants.
* @param[in,out] r a reference to a tgModelVisitor
*/
void tgCordeModel::onVisit(const tgModelVisitor& r) const
{
    r.render(*this);
}

 // Called from controller class, it makes the restLength get closer to preferredlength.
void tgCordeModel::moveMotors(double dt)
{
   // @todo add functions from muscle2P Bounded
    
    
    const double stiffness = m_string->getStiffness();
    // @todo: write invariant that checks this;
    assert(stiffness > 0.0);
    
    // Reverse the sign if restLength >= preferredLength
    // Velocity limiter
    double stepSize = m_config.targetVelocity * dt;
    // Acceleration limiter
    const double velChange = m_config.maxAcc * dt;
    const double actualLength = m_string->getActualLength();
    const double mostRecentVelocity = m_prevVelocity;
    
    /// @todo figure out how to re-calculate this, this is "too stiff"
    // First, change preferred length so we don't go over max tension
    if ((actualLength - m_preferredLength) * stiffness
            > m_config.maxTens)
    {
        m_preferredLength = actualLength - m_config.maxTens / stiffness;
    }
    
    double diff =  m_preferredLength - m_restLength;
    const double fabsDiff = std::abs(diff);
    
    // If below actual length, don't shorten any more
    if ((actualLength > m_config.minActualLength) || 
    (diff > 0))
    {
        if (abs(diff) > stepSize)
    {
        //Cap Velocity
      if (abs((diff/fabsDiff) * m_config.targetVelocity -
          mostRecentVelocity) >
          velChange)
      {
          // Cap Acceleration
          stepSize = velChange * dt;
      }
      m_restLength += (diff/fabsDiff)*stepSize;
    }
    else
    {
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
        m_restLength += diff;
    }
    }
    
     m_restLength =
      (m_restLength > m_config.minRestLength) ? m_restLength : m_config.minRestLength;
     #if (0)
     std::cout << "RL: " << m_restLength << " M2P RL: " << m_muscle->getRestLength() << std::endl;
     
     
     std::cout  << "RL: " << m_restLength
     << " Vel: " << (m_restLength  - m_muscle->getRestLength()) / dt 
     << " prev Vel: " << prevVel
     << " force " << (actualLength - m_restLength)*stiffness << std::endl;
     prevVel = (m_restLength  - m_muscle->getRestLength()) / dt ;
     #endif
     m_string->setRestLength(m_restLength);
    	
}

// @todo look into a base class implementation of this. Wouldn't be
// difficult with existing get functions
void tgCordeModel::tensionMinLengthController(const double targetTension,
										float dt)
{
	
}

void tgCordeModel::setRestLength(double newLength, float dt)
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
    /// @todo
    //assert(invariant());
    
}
  
const double tgCordeModel::getStartLength() const
{
	return m_startLength;
}

const double tgCordeModel::getCurrentLength() const 
{
	return m_string->getActualLength();
}

const double tgCordeModel::getTension() const
{
	
}

const double tgCordeModel::getRestLength() const
{
	return m_string->getRestLength();
}

/// Velocity here is set up by logHistory. Material strain is not considered.
const double tgCordeModel::getVelocity() const
{
	return m_prevVelocity;
}

void tgCordeModel::logHistory(const double dt)
{
	/// @todo assert about dt, etc.
	
	m_prevVelocity = (getRestLength () - m_prevLength) / dt;
	
	if (m_config.hist)
    {
        m_pHistory->lastLengths.push_back(m_string->getActualLength());
        m_pHistory->lastVelocities.push_back(m_prevVelocity );
        /// @todo do we need this?: m_pHistory->dampingHistory.push_back(m_muscle->getDamping());
        m_pHistory->restLengths.push_back(m_string->getRestLength());
        /// @todo m_pHistory->tensionHistory.push_back(m_string->getTension());
    }
    
    m_prevLength = m_string->getRestLength();
}
