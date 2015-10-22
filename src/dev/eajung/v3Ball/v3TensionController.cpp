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
 * @file v3TensionController.cpp
 * @brief Implementation of six strut tensegrity based from Berkeley's v3 Ball.
 * @author Erik Jung
 * @version 1.1.0
 * $Id$
 */

// This module
#include "v3TensionController.h"
// This application
#include "v3Model.h"
// This library
#include "core/tgBasicActuator.h"
#include "controllers/tgTensionController.h"
// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <cassert>
#include <math.h>
#include <vector>
#include <stdexcept>

using namespace std;

v3TensionController::v3TensionController(const double initialLength, double timestep, btVector3 goalTrajectory) :
		m_initialLengths(initialLength),
		m_totalTime(0.0)
{
  this->initPos = btVector3(0,0,0);
}

v3TensionController::~v3TensionController()
{
	std::size_t n = m_controllers.size();
	for(std::size_t i = 0; i < n; i++)
	{
		delete m_controllers[i];
	}
	m_controllers.clear();
}

void v3TensionController::onSetup(v3Model& subject)
{
    /*const std::vector<tgBasicActuator*> actuators = subject.getAllActuators();
    for (size_t i = 0; i < actuators.size(); ++i)
    {
        tgBasicActuator * const pActuator = actuators[i];
        assert(pActuator != NULL);
        tgTensionController* m_tensController = new tgTensionController(pActuator, m_tension);
        m_controllers.push_back(m_tensController);
    }*/

   this->initPos=endEffectorCOM(subject);
   this->m_totalTime = 0.0;

}

void v3TensionController::onStep(v3Model& subject, double dt)
{
	if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    /*else
    {
        std::size_t n = m_controllers.size();
		for(std::size_t i = 0; i < n; i++)
        {
            m_controllers[i]->control(dt, m_tension);
        }
    }*/

    m_totalTime += dt;
  
    btVector3 ee = endEffectorCOM(subject);
    std::cout << m_totalTime << " " << ee.getX() << " " << ee.getY() << " " << ee.getZ() << std::endl;


}

btVector3 v3TensionController::endEffectorCOM(v3Model& subject) {
	const std::vector<tgRod*> endEffector = subject.find<tgRod>("endeffector");
	assert(!endEffector.empty());
	return endEffector[0]->centerOfMass();
}


