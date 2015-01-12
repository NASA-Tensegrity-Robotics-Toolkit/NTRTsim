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
 * @file tgTensionController.cpp
 * @brief Implementation of the tgTensionController class
 * @author Brian Mirletz
 * @date December 2014
 * $Id$
 */

#include "tgTensionController.h"

#include "core/tgBasicActuator.h"
#include "core/tgSpringCable.h"
#include "core/tgCast.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <cstddef> // NULL keyword

tgTensionController::tgTensionController(tgBasicActuator* controllable, double setPoint) :
m_sca(controllable),
tgBasicController(controllable, setPoint)
{
	assert(controllable != NULL);
}

tgTensionController::~tgTensionController()
{
    m_sca = NULL;
}
	
void tgTensionController::control(double dt)
{
	if (dt <= 0.0)
	{
		throw std::runtime_error ("Timestep must be positive.");
	}	
	
    const tgSpringCable* m_springCable = m_sca->getSpringCable();
    
    const double stiffness = m_springCable->getCoefK();
    // @todo: write invariant that checks this;
    assert(stiffness > 0.0);
    
    const double currentTension = m_springCable->getTension();
    const double delta = m_setPoint - currentTension;
    double diff = delta / stiffness; 
    const double currentLength = m_springCable->getRestLength();
    
    double newLength = m_sca->getRestLength() - diff;
    
    // Safety check
    newLength = newLength < 0.0 ? 0.0 : newLength;
    
	m_sca->setControlInput(newLength, dt);
}

void tgTensionController::control(double dt, double setPoint, double sensorData)
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

void tgTensionController::control(tgBasicActuator& sca, double dt, double setPoint)
{
    if (dt <= 0.0)
	{
		throw std::runtime_error ("Timestep must be positive.");
	}	
	
    const tgSpringCable* m_springCable = sca.getSpringCable();
    
    const double stiffness = m_springCable->getCoefK();
    // @todo: write invariant that checks this;
    assert(stiffness > 0.0);
    
    const double currentTension = m_springCable->getTension();
    const double delta = setPoint - currentTension;
    double diff = delta / stiffness; 
    const double currentLength = m_springCable->getRestLength();
    
    double newLength = sca.getRestLength() - diff;
    
    // Safety check
    newLength = newLength < 0.1 ? 0.1 : newLength;
    
	sca.setControlInput(newLength, dt);
}
