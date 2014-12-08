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
 * @file tgBasicController.cpp
 * @brief Implementation of the tgBasicController base class
 * @author Brian Mirletz
 * @date December 2014
 * $Id$
 */

#include "tgBasicController.h"

#include "core/tgControllable.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <cstddef> // NULL keyword

tgBasicController::tgBasicController(tgControllable* controllable, double setPoint) :
m_setPoint(setPoint),
m_controllable(controllable)
{
	assert(controllable != NULL);
}

tgBasicController::~tgBasicController()
{
	m_controllable = NULL;
}
	
void tgBasicController::control(double dt)
{
	if (dt <= 0.0)
	{
		throw std::runtime_error ("Timestep must be positive.");
	}	
	
	m_controllable->setControlInput(m_setPoint);
}
	
void tgBasicController::control(double dt, double setPoint, double sensorData)
{
	if (dt <= 0.0)
	{
		throw std::runtime_error ("Timestep must be positive.");
	}
	
	// Suppress compiler warning for unused variable
	(void)sensorData;
	
	setNewSetPoint(setPoint);
	control(dt);
}
	
void tgBasicController::setNewSetPoint(double newSetPoint)
{
	m_setPoint = newSetPoint;
}


