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
 * @file tgRBString.cpp
 * @brief Contains the definition of class tgRBString. A string with
 * small rigid bodies to create contact dynamics.
 * @author Brian Tietz
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgRBString.h"
#include "core/tgLinearString.h"

#include "core/tgCast.h"

// The C++ Standard Library
#include <cmath>
#include <stdexcept>

/**
 * Dummy constructor to make the compiler happy.
 * @todo remove this
 */
tgRBString::Config::Config()
{
    throw std::invalid_argument("Failed to provide arguments to tgRBString::Config");
}

tgRBString::Config::Config( std::size_t segments,
                const tgRod::Config& rodConf,
                const tgLinearString::Config& stringConf,
                double minTotalLength) :
m_segments(segments),
m_rodConfig(rodConf),
m_stringConfig(stringConf),
m_minTotalLength(minTotalLength)
{
}

// @todo consider storing other confing info as a member variable
tgRBString::tgRBString(const tgTags& tags,
           tgRBString::Config& config,
           double restLength) :
tgBaseString(tags, config.m_stringConfig, restLength, restLength),
m_config(config)
{
}

void tgRBString::setup(tgWorld& world)
{
    // These occur after the build process
    allSegments = tgCast::filter<tgModel, tgRod> (getDescendants());
    
    assert(allSegments.size() == m_config.m_segments);
    
    allMuscles = this->find<tgLinearString> ("muscle");
    // Should we assert something here?
    
    // Consider making this an assert since tensionMinLength also
    // Depends on it
    if (m_config.m_segments >= 2)
    {   
        const double musclesPerSegment = (double) allMuscles.size() / 
                                (double) (m_config.m_segments - 1);
        /* True if all muscles are homogenous, which should be true
         * given tgRBStringInfo's use of configs. Should this include
         * RB segment mass?
         */
        m_effectiveStiffness =  m_config.m_stringConfig.stiffness *
                                musclesPerSegment / 
                                (double) (m_config.m_segments - 1);
    }
    else
    {
        // Infinite stiffness means its a single RB, which has no tension
        m_effectiveStiffness = 0;
    }
    // All the heavy lifting is done by info
    tgModel::setup(world);
    logHistory(0.0);
    
}

void tgRBString::teardown()
{
    tgModel::teardown();
}

void tgRBString::step(double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive.");
    }
    else
    { 

        logHistory(dt);
        tgModel::step(dt);  // Step any children
        
#if (1)
            std::cout << "Tension: " << getTension() <<
            " Calculated Rest: " << getRestLength() << 
            " Actual length: " << getCurrentLength() << std::endl;
#endif 
    }

}

//Does this make myModel an observer as well??
void tgRBString::changeMuscles (double lengthPercent, double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive.");
    }
    else
    {
        assert(lengthPercent > 0.0);
        for( int i = 0; i < allMuscles.size(); i++){
            const double rl = allMuscles[i]->getRestLength();
            
            assert(rl > 0.0);
            #if (0)
            std::cout << "Indiv Muscle " << rl * lengthPercent << std::endl;
            #endif
            
            allMuscles[i]->setRestLength(rl * lengthPercent, dt);
        }
    }
}

void tgRBString::moveMotors(double dt)
{
    // @todo add functions from muscle2P Bounded
    
    // Reverse the sign if restLength >= preferredLength
    // Velocity limiter
    double stepSize = m_config.m_stringConfig.targetVelocity * dt;
    // Acceleration limiter
    const double velChange = m_config.m_stringConfig.maxAcc * dt;
    const double actualLength = getCurrentLength();
    const double mostRecentVelocity = m_pHistory->lastVelocities.back();
    m_restLength = getRestLength();
    
    double diff = m_preferredLength - m_restLength;
    const double fabsDiff = std::abs(diff);
    
    // If below actual length, don't shorten any more
    if ((actualLength > m_config.m_stringConfig.minActualLength) || 
    (diff > 0))
    {
        if (abs(diff) > stepSize)
    {
        //Cap Velocity
      if (std::abs((diff/fabsDiff) * m_config.m_stringConfig.targetVelocity -
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
        if (std::abs(diff/dt - mostRecentVelocity) > velChange)
        {
        // Cap Acceleration
        if (diff != 0) 
        {
          diff = (diff/fabsDiff) * velChange * dt;
        }
        else
        { 
            // If m_prevVelocity was zero, it would be smaller than
            // velChange. Therefore preVelocity is valid for figuring
            // out direction
          diff = -(mostRecentVelocity / std::abs(mostRecentVelocity)) *
                 velChange * dt;
        }
        }
        m_restLength += diff;
    }
    }
    
    // Ensure we aren't going below min rest length
    if(m_restLength >= m_config.m_stringConfig.minRestLength)
    {
        changeMuscles(m_restLength / getRestLength(), dt);
    }
    
}
    
void tgRBString::tensionMinLengthController(const double targetTension,
                                            float dt)
{
    const double stiffness = m_effectiveStiffness;
    // @todo: write invariant that checks this;
    assert(stiffness > 0.0);
    
    const double currentTension = getTension();
    const double delta = targetTension - currentTension;
    double diff = delta / stiffness; 
    m_restLength = getRestLength();
    
    const double newLength = m_restLength - diff;

    m_preferredLength =
      (newLength > m_config.m_stringConfig.minRestLength) ? 
                    newLength : m_config.m_stringConfig.minRestLength;
#if (0)
    std::cout << "m_preferred: " << m_preferredLength << std::endl;
#endif
    moveMotors(dt);
}
    
const double tgRBString::getStartLength() const
{
     return m_pHistory->lastLengths.front();
}
    
const double tgRBString::getCurrentLength() const
{
    double currLength = 0;
    
    // This doesn't consider anchors
    std::size_t n = allSegments.size() - 1;
    for (std::size_t i = 0; i < n; i++)
    {
        const btVector3 rodCenterOfMass1 = allSegments[i]->centerOfMass();
        const btVector3 rodCenterOfMass2 = allSegments[i + 1]->centerOfMass();
        
        currLength += (rodCenterOfMass2 - rodCenterOfMass1).length();
    }
    
    // Add the distance from the COM to the ends of the rods
    currLength += allSegments[0]->length();
    
    return currLength;
}

const double tgRBString::getTension() const
{
    const double tension = (getCurrentLength() - getRestLength())
                            * m_effectiveStiffness;
    return (tension < 0) ? 0.0 : tension;
}


// How does this relate to m_restLength?
const double tgRBString::getRestLength() const
{
    double muscleRest = 0;
    double rodRest = 0;
    
    std::size_t segSize = allSegments.size();
    for (std::size_t i = 0; i < segSize; i++)
    {
        rodRest += allSegments[i]->length();
    }
    
    std::size_t mSize = allMuscles.size();
    for (std::size_t i = 0; i < mSize; i++)
    {
        muscleRest += allMuscles[i]->getRestLength();
    }
    
    // This assumes all four muscles for a given segment have the 
    // same RL, or that averaging is appropreate
    muscleRest /= ((double) mSize / (double) (segSize - 1));
    
    return rodRest + muscleRest;
}

const double tgRBString::getVelocity() const
{
    return m_pHistory->lastVelocities.back();
}

const double tgRBString::computeVelocity(const double dt) const
{   
    // Check to make sure we're calling this at the start of or after
    // a log history step
    assert (m_pHistory->lastLengths.size() == m_pHistory->lastVelocities.size());
    double vel;
    if (dt > 0.0)
    {
        vel = (getCurrentLength() - m_pHistory->lastLengths.back()) / dt;
    }
    else if (dt == 0.0)
    {
        // For zero velocity at startup.
        vel = 0.0;
    }
    else
    {
        throw std::invalid_argument("dt is negitive.");
    }
    return vel;
}

// Private function, dt should have already been checked
void tgRBString::logHistory(const double dt)
{
    const double currentVelocity = computeVelocity(dt);
    // Needed for rendering, so not optional
    m_pHistory->lastLengths.push_back(getCurrentLength());
    // Needed for moveMotors
    m_pHistory->lastVelocities.push_back(currentVelocity);
    if (m_config.m_stringConfig.hist)
    {
        // Damping history is difficult to compute, so we're leaving it out
        m_pHistory->restLengths.push_back(getRestLength());
        m_pHistory->tensionHistory.push_back(getTension());
    }
}
