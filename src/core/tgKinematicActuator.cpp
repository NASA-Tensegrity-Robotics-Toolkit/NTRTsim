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
 * @file tgKinematicActuator.cpp
 * @brief Contains the definitions of members of class tgKinematicActuator
 * @author Brian Mirletz
 * $Id$
 */

// This Module
#include "tgKinematicActuator.h"
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

tgKinematicActuator::Config::Config(double s,
									double d,
									double p,
									double rad,
									double moFric,
									double moInert,
									bool back,
									bool h,
									double mf,
									double tVel,
									double mnAL,
									double mnRL,
									double rot) :
tgSpringCableActuator::Config::Config(s, d, p, h,
							   mf, tVel, mnAL, mnRL, rot),
radius(rad),
motorFriction(moFric),
motorInertia(moInert),
backdrivable(back),
maxOmega(tVel / rad),
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

void tgKinematicActuator::constructorAux()
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
tgKinematicActuator::tgKinematicActuator(tgBulletSpringCable* muscle,
                   const tgTags& tags,
                   tgKinematicActuator::Config& config) :
    m_motorVel(0.0),
    m_motorAcc(0.0),
    m_appliedTorque(0.0),
    m_config(config),
    tgSpringCableActuator(muscle, tags, config)
{
    constructorAux();

    // Postcondition
    assert(invariant());
}

tgKinematicActuator::~tgKinematicActuator()
{
    //std::cout << "deleting kinematic spring cable" << std::endl;
    // Should have already torn down.
}
    
void tgKinematicActuator::setup(tgWorld& world)
{
    // This needs to be called here in case the controller needs to cast
    notifySetup();
    tgModel::setup(world);
}

void tgKinematicActuator::teardown()
{
    // Do not notify teardown. The controller has already been deleted.
    tgModel::teardown();
}
    
void tgKinematicActuator::step(double dt) 
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgKinematicActuator::step");
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
        m_springCable->step(dt);
        logHistory();  
        tgModel::step(dt);
    }
    
    // Reset and wait for next control input
    m_desiredTorque = 0.0;
}

void tgKinematicActuator::onVisit(const tgModelVisitor& r) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgKinematicActuator::onVisit");
#endif //BT_NO_PROFILE	
    r.render(*this);
}
    
void tgKinematicActuator::logHistory()
{
    m_prevVelocity = getVelocity();

    if (m_config.hist)
    {
        m_pHistory->lastLengths.push_back(m_springCable->getActualLength());
        m_pHistory->lastVelocities.push_back(m_motorVel);
        m_pHistory->dampingHistory.push_back(m_springCable->getDamping());
        m_pHistory->restLengths.push_back(m_springCable->getRestLength());
        m_pHistory->tensionHistory.push_back(m_appliedTorque);
    }
}
    
const double tgKinematicActuator::getVelocity() const
{
    return m_motorVel * m_config.radius;
}

void tgKinematicActuator::integrateRestLength(double dt)
{
	double tension = getTension();
	m_appliedTorque = getAppliedTorque(m_desiredTorque);
	// motorVel will always cause opposite acc, but tension can only
	// cause lengthening (positive Acc)
	m_motorAcc = (m_appliedTorque - m_config.motorFriction * m_motorVel 
					+ tension * m_config.radius) / m_config.motorInertia;
	
	if (!m_config.backdrivable && m_motorAcc * m_appliedTorque <= 0.0)
	{
		// Stop undesired lengthing if the motor is not backdrivable
		m_motorVel = m_motorVel + m_motorAcc * dt > 0.0 ? 0.0 : m_motorVel + m_motorAcc * dt;
	}
	else
	{
		m_motorVel += m_motorAcc * dt;
	}
	
	// semi-implicit Euler integration
	m_restLength += m_config.radius * m_motorVel * dt; 
	
	/// @todo check min actual length somewhere
	
	m_restLength =
	(m_restLength > m_config.minRestLength) ? m_restLength : m_config.minRestLength;

	m_springCable->setRestLength(m_restLength);
}

double tgKinematicActuator::getAppliedTorque(double desiredTorque) const
{ 
	double maxTorque = m_config.maxTens * m_config.radius * 
						(1.0 - m_config.radius * abs(m_motorVel) / m_config.targetVelocity);
	
	maxTorque = maxTorque < 0.0 ? 0.0 : maxTorque;
	
	return abs(desiredTorque) < maxTorque ? desiredTorque : 
		desiredTorque / abs(desiredTorque) * maxTorque;
}

void tgKinematicActuator::setControlInput(double input)
{
	m_desiredTorque = input;
}

const tgSpringCableActuator::SpringCableActuatorHistory& tgKinematicActuator::getHistory() const
{
    return *m_pHistory;
}

bool tgKinematicActuator::invariant() const
{
    return
      (m_springCable != NULL) &&
      (m_pHistory != NULL) && 
      (m_config.targetVelocity >= 0.0) &&
      (m_config.minActualLength >= 0.0);
}
