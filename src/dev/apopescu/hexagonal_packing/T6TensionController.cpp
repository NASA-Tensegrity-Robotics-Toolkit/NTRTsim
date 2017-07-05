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
 * @file T6TensionController.cpp
 * @brief Implementation of six strut tensegrity.
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

// This module
#include "T6TensionController.h"
// This application
#include "T6Model.h"
// This library
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <iostream>
#include <math.h>

T6TensionController::T6TensionController(const double tension) :
    m_tension(tension)
{
    if (tension < 0.0)
    {
        throw std::invalid_argument("Negative tension");
    }
}

T6TensionController::~T6TensionController()
{
	std::size_t n = m_controllers.size();
    for(std::size_t i = 0; i < n; i++)
    {
        delete m_controllers[i];
    }
    m_controllers.clear();
}	

void T6TensionController::onSetup(T6Model& subject)
{
  m_controllers.clear(); //clear vector of controllers
  
    std::cout << "Tension controller started!" << std::endl;
    const std::vector<tgBasicActuator*> actuators = subject.getAllActuators();
    for (size_t i = 0; i < actuators.size(); ++i)
    {
        // We are using tgBasicActuator, which controls rest length of each string/spring directly.
        tgBasicActuator * const pActuator = actuators[i];

        assert(pActuator != NULL);
        // Create a tension controller for each muscle.
        tgTensionController* m_tensController = new tgTensionController(pActuator, m_tension);
        m_controllers.push_back(m_tensController);
    }
    globalTime = 0;
}

void T6TensionController::onStep(T6Model& subject, double dt)
{
	if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        globalTime += dt;
        // Top/bottom triangles
        double tbtens = 1000;
        if (globalTime > 3) tbtens = 1500;
		for(std::size_t i = 0; i <= 2; i++) m_controllers[i]->control(dt, tbtens);
		for(std::size_t i = 6; i <= 8; i++) m_controllers[i]->control(dt, tbtens);
		// Vertical strings
		for(std::size_t i = 3; i <= 5; i++) m_controllers[i]->control(dt, 1000);
		for(std::size_t i = 9; i <= 11; i++) m_controllers[i]->control(dt, 1000);
		// Middle ring / saddle cables
		for(std::size_t i = 12; i <= 17; i++) m_controllers[i]->control(dt, 1500);
		// Vertical tensioners
		for(std::size_t i = 18; i <= 23; i++) m_controllers[i]->control(dt, 300);
	}
}
