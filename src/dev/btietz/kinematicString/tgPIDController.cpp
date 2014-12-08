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
 * @file tgPIDController.cpp
 * @brief Implementation of the tgPIDController class
 * @author Brian Mirletz
 * @date December 2014
 * $Id$
 */


#include "tgPIDController.h"

#include "tgControllable.h"

// The C++ Standard Library
#include <stdexcept>
#include <cassert>

tgPIDController::Config::Config(double p,
									double i,
									double d,
									double setPoint) :
kP(p),
kI(i),
kD(d),
startingSetPoint(setPoint)
{
	if (p < 0.0)
    {
        throw std::invalid_argument("Position gain is negative");
    }
    else if (i < 0.0)
    {
        throw std::invalid_argument("Integral gain is negative.");
    }
    else if (d < 0.0)
    {
        throw std::invalid_argument("Derivative gain is negative.");
    }
}
	

tgPIDController::tgPIDController(tgControllable* controllable, tgPIDController::Config config) :
m_sensorData(0.0),
m_prevError(0.0),
m_intError(0.0),
m_config(config),
tgBasicController(controllable, config.startingSetPoint)
{
	assert(controllable != NULL);
}

tgPIDController::~tgPIDController()
{
	// tgBasicController owns m_controllable
}
	
void tgPIDController::control(double dt)
{
	double error = m_setPoint - m_sensorData;
	
	/**
	 * @todo since we have access to prevError, should we integrate by
	 * another method (ie (m_prevError + error) / 2.0 * dt)?
	 */
	m_intError += error * dt;
	double dError = (error - m_prevError) / dt;
	double result = m_config.kP * error + m_config.kI * m_intError +
					m_config.kD * dError;
	
	m_controllable->setControlInput(result);
	
	m_prevError = error;
}
	
void tgPIDController::control(double dt, double setPoint, double sensorData)
{
	setSensorData(sensorData);
	m_setPoint = setPoint;
	control(dt);
}

void tgPIDController::setSensorData(double sensorData)
{
	/// @todo - are there any sanity checks we can enforce here?
	m_sensorData = sensorData;
}
