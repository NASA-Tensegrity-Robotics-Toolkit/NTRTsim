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
 * @file RPTensionController.cpp
 * @brief Implementation of six strut tensegrity.
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

#include <iostream>
// This module
#include "RPTensionController.h"
// This application
#include "RPModel.h"
// This library
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>

using namespace std;

RPTensionController::RPTensionController(const double tension) :
    m_tension(tension)
{
    if (tension < 0.0)
    {
        throw std::invalid_argument("Negative tension");
    }
}

RPTensionController::~RPTensionController()
{
  
    std::size_t n = m_controllers.size();
    for(std::size_t i = 0; i < n; i++)
    {
        delete m_controllers[i];
    }
  
    m_controllers.clear();
   
}	

void RPTensionController::onSetup(RPModel& subject)
{
  m_controllers.clear();
    const std::vector<tgBasicActuator*> actuators = subject.getAllActuators();
    //if(m_controllers.size() != 36){
    for (size_t i = 0; i < actuators.size(); ++i)
    {
        tgBasicActuator * const pActuator = actuators[i];
        assert(pActuator != NULL);
        tgTensionController* m_tensController = new tgTensionController(pActuator, m_tension);
        m_controllers.push_back(m_tensController);
    }
    //}

}

void RPTensionController::onStep(RPModel& subject, double dt)
{
	if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        std::size_t n = m_controllers.size();
        // int m=m_controllers.size();
	// cout<<m;
	// 	for(std::size_t i = 0; i < 23; i++)
        // {
        //     m_controllers[i]->control(dt, m_tension);
        // }

	//cout<<m_controllers.size()<< " " << m_controllers[0] << endl;

	//uncomment desired controllers manually...
	m_controllers[0]->control(dt,m_tension);
	m_controllers[1]->control(dt,m_tension);
	//m_controllers[2]->control(dt,m_tension);
	//m_controllers[3]->control(dt,m_tension);
	//m_controllers[4]->control(dt,m_tension);
	//m_controllers[5]->control(dt,m_tension);
	//m_controllers[6]->control(dt,m_tension);
	//m_controllers[7]->control(dt,m_tension);
	//m_controllers[8]->control(dt,m_tension);
	//m_controllers[9]->control(dt,m_tension);
	//m_controllers[10]->control(dt,m_tension);
	//m_controllers[11]->control(dt,m_tension);
	//m_controllers[12]->control(dt,m_tension);
	//m_controllers[13]->control(dt,m_tension);
	//m_controllers[14]->control(dt,m_tension);
	//m_controllers[15]->control(dt,m_tension);
	//m_controllers[16]->control(dt,m_tension);
	//m_controllers[17]->control(dt,m_tension);
	//m_controllers[18]->control(dt,m_tension);
	//m_controllers[20]->control(dt,m_tension);
	//m_controllers[21]->control(dt,m_tension);
	//m_controllers[22]->control(dt,m_tension);
	//m_controllers[23]->control(dt,m_tension);
	//m_controllers[24]->control(dt,m_tension);
	//m_controllers[25]->control(dt,m_tension);
	//m_controllers[26]->control(dt,m_tension);
	//m_controllers[27]->control(dt,m_tension);
	//m_controllers[28]->control(dt,m_tension);
	//m_controllers[29]->control(dt,m_tension);
	//m_controllers[30]->control(dt,m_tension);
	//m_controllers[31]->control(dt,m_tension);
	//m_controllers[32]->control(dt,m_tension);
	//m_controllers[33]->control(dt,m_tension);
	//m_controllers[34]->control(dt,m_tension);
	//m_controllers[35]->control(dt,m_tension);
	}
}
