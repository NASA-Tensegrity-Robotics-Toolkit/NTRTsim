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
 * @file tgKinematicString.cpp
 * @brief Contains the definitions of members of class tgKinematicString
 * @author Brian Mirletz
 * $Id$
 */

// This Module
#include "tgKinematicString.h"
// The NTRT Core libary
#include "core/tgBulletSpringCable.h"
#include "core/tgModelVisitor.h"
#include "core/tgWorld.h"
// The Bullet Physics Library
#include "LinearMath/btQuickprof.h"

// The C++ Standard Library
#include <cmath>
#include <deque> // For history
#include <iostream>
#include <stdexcept>

using namespace std;

tgKinematicString::Config::Config(double s,
									double d,
									double p,
									double rad,
									double moFric,
									double moInert,
									bool back,
									bool h,
									double mf,
									double tVel,
									double mxAcc,
									double mnAL,
									double mnRL,
									double rot) :
tgBaseString::Config::Config(s, d, p, h,
							   mf, tVel, mxAcc, mnAL, mnRL, rot),
radius(rad),
motorFriction(moFric),
motorInertia(moInert),
backdrivable(back),
maxOmega(tVel / rad),
maxDOmega(mxAcc / rad),
maxTorque(mf / rad)
{
	if (rad <= 0.0)
    {
        throw std::invalid_argument("Radius is non-positive");
    }
    else if (moFric < 0.0)
    {
        throw std::invalid_argument("Motor friction is negative.");
    }
    else if (moInert <= 0.0)
    {
        throw std::invalid_argument("Motor inertia is non-positive");
    }
}

void tgKinematicString::constructorAux()
{
  // Precondition
    assert(m_pHistory != NULL);
    prevVel = 0.0;
    if (m_muscle == NULL)
    {
        throw std::invalid_argument("Pointer to tgBulletSpringCable is NULL.");
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
tgKinematicString::tgKinematicString(tgBulletSpringCable* muscle,
                   const tgTags& tags,
                   tgKinematicString::Config& config) :
    m_muscle(muscle),
    m_motorVel(0.0),
    m_motorAcc(0.0),
    m_config(config),
    tgBaseString(tags, config, muscle->getRestLength(), muscle->getActualLength())
{
    constructorAux();

    // Postcondition
    assert(invariant());
    assert(m_muscle == muscle);
    assert(m_preferredLength == m_restLength);
}

tgKinematicString::~tgKinematicString()
{
    //std::cout << "deleting linear string" << std::endl;
    // Should have already torn down.
    delete m_muscle;
}
    
void tgKinematicString::setup(tgWorld& world)
{
    notifySetup();
    tgModel::setup(world);
}

void tgKinematicString::teardown()
{
    // Do not notify teardown. The controller has already been deleted.
    tgModel::teardown();
}
    
void tgKinematicString::step(double dt) 
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgKinematicString::step");
#endif //BT_NO_PROFILE   	
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive.");
    }
    else
    {   
        // Want to update any controls before applying forces
        notifyStep(dt); 
        // Adjust rest length based on muscle dynamics
        integrateRestLength(dt);
        m_muscle->calculateAndApplyForce(dt);
        logHistory();  
        tgModel::step(dt);
    }
    
    // Reset and wait for next control input
    m_desiredTorque = 0.0;
}

void tgKinematicString::onVisit(const tgModelVisitor& r) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgKinematicString::onVisit");
#endif //BT_NO_PROFILE	
    r.render(*this);
}
    
void tgKinematicString::logHistory()
{
    m_prevVelocity = getVelocity();

    if (m_config.hist)
    {
        m_pHistory->lastLengths.push_back(m_muscle->getActualLength());
        m_pHistory->lastVelocities.push_back(m_muscle->getVelocity());
        m_pHistory->dampingHistory.push_back(m_muscle->getDamping());
        m_pHistory->restLengths.push_back(m_muscle->getRestLength());
        m_pHistory->tensionHistory.push_back(m_muscle->getTension());
    }
}
    
const double tgKinematicString::getStartLength() const
{
    return m_startLength;
}
    
const double tgKinematicString::getCurrentLength() const
{
    return m_muscle->getActualLength();
}  

const double tgKinematicString::getTension() const
{
    return m_muscle->getTension();
}
    
const double tgKinematicString::getRestLength() const
{
    return m_muscle->getRestLength();
}

const double tgKinematicString::getVelocity() const
{
    return m_motorVel * m_config.radius;
}

void tgKinematicString::setRestLength(double newLength, float dt)
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

void tgKinematicString::setPrefLength(double newLength)
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

void tgKinematicString::integrateRestLength(double dt)
{
	double tension = getTension();
	double appliedTorque = getAppliedTorque(m_desiredTorque);
	// motorVel will always cause opposite acc, but tension can only
	// cause lengthening (positive Acc)
	m_motorAcc = (appliedTorque - m_config.motorFriction * m_motorVel 
					+ tension * m_config.radius) / m_config.motorInertia;
	
	if (!m_config.backdrivable && m_motorAcc * appliedTorque <= 0.0)
	{
		// Stop undesired lengthing if the motor is not backdrivable
		m_motorVel = m_motorVel + m_motorAcc * dt > 0.0 ? 0.0 : m_motorVel + m_motorAcc * dt;
	}
	else
	{
		m_motorVel += m_motorAcc * dt;
	}
	
	// semi-implicit Euler integration
	m_restLength += m_motorVel * dt; 
	
	/// @todo check min actual length somewhere
	
	m_restLength =
	(m_restLength > m_config.minRestLength) ? m_restLength : m_config.minRestLength;

	m_muscle->setRestLength(m_restLength);
}

double tgKinematicString::getAppliedTorque(double desiredTorque) const
{ 
	double maxTorque = m_config.maxTens * m_config.radius * 
						(1.0 - m_config.radius * abs(m_motorVel) / m_config.targetVelocity);
	
	maxTorque = maxTorque < 0.0 ? 0.0 : maxTorque;
	
	return abs(desiredTorque) < maxTorque ? desiredTorque : 
		desiredTorque / abs(desiredTorque) * maxTorque;
}

void tgKinematicString::moveMotors(double dt)
{
	double error = m_preferredLength - m_restLength;
	// Simple p control based on known motor parameters
    m_desiredTorque = 100.0 * error * m_config.motorInertia / m_config.radius;
}

void tgKinematicString::tensionMinLengthController(const double targetTension,
                      float dt)
{

    const double stiffness = m_muscle->getCoefK();
    // @todo: write invariant that checks this;
    assert(stiffness > 0.0);
    
    const double currentTension = m_muscle->getTension();
    const double delta = targetTension - currentTension;
    
    m_desiredTorque = delta * m_config.radius;
}

void tgKinematicString::setControlInput(double input)
{
	m_desiredTorque = input;
}

const tgBaseString::BaseStringHistory& tgKinematicString::getHistory() const
{
    return *m_pHistory;
}

bool tgKinematicString::invariant() const
{
    return
      (m_muscle != NULL) &&
      (m_pHistory != NULL) && 
      (m_config.targetVelocity >= 0.0) &&
      (m_config.maxAcc >= 0.0) &&
      (m_config.minActualLength >= 0.0) &&
      (m_preferredLength >= 0.0);
}
