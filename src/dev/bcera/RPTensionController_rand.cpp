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
  const std::vector<tgBasicActuator*> actuators = subject.getAllActuators();
  for (size_t i = 0; i < actuators.size(); ++i)
    {
      tgBasicActuator * const pActuator = actuators[i];
      assert(pActuator != NULL);
      tgTensionController* m_tensController = new tgTensionController(pActuator, m_tension);
      m_controllers.push_back(m_tensController);
    }
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
      //for(std::size_t i = 0; i < tensions.size(); i++)
      //{
          //m_controllers[i+24]->control(dt, tensions[i],0);
	  //tgTensionController::control(m_controllers[i],dt,tensions[i]);
      //}
      
      //define random indices to select 1st, 2nd and 3rd actuators at random
      //2nd and 3rd indices are placed in a while loop that prevents the
      //indices from repeating
      
      int t1= rand() %12 +24;
      int t2= rand() %12 +24;
      int t3= rand() %12 +24;
      while (t1==t2) {
	t2= rand() %12 +24;
      }
      while (t1==t3 || t2==t3) {
	t3= rand() %12 +24;
      }

      //uncomment one or two for either randomly actuated
      //pairs or randomly actuated triplets respectively
      
      m_controllers[t1]->control(dt,m_tension);
      //m_controllers[t2]->control(dt,m_tension);
      //m_controllers[t3]->control(dt,m_tension);
    } 
}
