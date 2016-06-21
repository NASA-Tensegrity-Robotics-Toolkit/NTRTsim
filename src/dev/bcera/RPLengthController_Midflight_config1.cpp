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
#include "RPLengthController.h"
// This application
#include "RPModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgCast.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <time.h>

using namespace std;

RPLengthController::RPLengthController(const double length) :
  m_length(length)
{
  if (length < 0.0)
    {
      throw std::invalid_argument("Negative length");
    }
}

RPLengthController::~RPLengthController()
{
  std::size_t n = m_controllers.size();
  for(std::size_t i = 0; i < n; i++)
    {
      delete m_controllers[i];
    }
  m_controllers.clear();
  rand_lengths.clear();
}	

void RPLengthController::onSetup(RPModel& subject)
{
  globalTime = 0;
  toggle = 0;
  m_controllers.clear(); 
  rand_lengths.clear();
  srand(time(NULL));
  srand48(time(NULL));
  const std::vector<tgBasicActuator*> actuators = subject.getAllActuators();
  for (size_t i = 0; i < actuators.size(); ++i)
    {
      tgBasicActuator * const pActuator = actuators[i];
      assert(pActuator != NULL);
      tgBasicController* m_lenController = new tgBasicController(pActuator, m_length);
      m_controllers.push_back(m_lenController);
      rand_lengths.push_back(pActuator->getRestLength()); 
    }
  //Lengthen lower cables
  rand_lengths[0] = rand_lengths[0]*2;
  rand_lengths[2] = rand_lengths[0]*2;
  rand_lengths[16] = rand_lengths[0]*2;
  //Shorten lower cables
  rand_lengths[13] = rand_lengths[0]*0.5;
  rand_lengths[15] = rand_lengths[0]*0.5;
  rand_lengths[23] = rand_lengths[0]*0.5;
}

void RPLengthController::onStep(RPModel& subject, double dt)
{
  if (dt <= 0.0)
    {
      throw std::invalid_argument("dt is not positive");
    }
  else
    {
      globalTime += dt;
      std::size_t n = m_controllers.size();
      // int m=m_controllers.size();
      //for(std::size_t i = 0; i < tensions.size(); i++)
      //{
      //m_controllers[i+24]->control(dt, tensions[i],0);
      //tgTensionController::control(m_controllers[i],dt,tensions[i]);
      //}
	
      //uncomment one or two for either randomly actuated
      //pairs or randomly actuated triplets respectively
	
      for(int p=0; p<rand_lengths.size(); p++){
	m_controllers[p]->control(dt,rand_lengths[p]);
      }
      //std::cout << rand_lengths[1] << " ";
      /*
	m_controllers[t1]->control(dt,rand_lengths[t1]);
	m_controllers[t2]->control(dt,rand_lengths[t2]);
	m_controllers[t3]->control(dt,rand_lengths[t3]);
      */
    }
  if(globalTime > 4){
    if(toggle==0){
      cout << endl << "Activating Cable Motors (Randomized Lengths) -------------------------------------" << endl;
      toggle = 1;
    }
    const std::vector<tgBasicActuator*> actuators = subject.getAllActuators();
    for(int i = 0; i<actuators.size(); i++){
      actuators[i]->moveMotors(dt);
      //cout << (double)rand_lengths[i] << " ";
    }
  }

}
