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
 * @file RestlengthController.cpp
 * @brief Simple Controller Implementation for Three Bar Tensegrity
 * @author Brian Cera
 * @version 1.0.0
 * $Id$
 */

#include <iostream>
// This module
#include "LengthController.h"
// This application
#include "tgModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgCast.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <time.h>

using namespace std;

LengthController::LengthController(const double length) :
  m_length(length)
{
  if (length < 0.0)
    {
      throw std::invalid_argument("Negative length");
    }
}

LengthController::~LengthController()
{
  std::size_t n = m_controllers.size();
  for(std::size_t i = 0; i < n; i++)
    {
      delete m_controllers[i];
    }
  m_controllers.clear();
  rand_lengths.clear();
}	

void LengthController::onSetup(Model& subject)
{
  globalTime = 0;
  toggle = 0;
  m_controllers.clear(); //clear vector of controllers
  rand_lengths.clear(); //vector of randomized restlengths
  //set seeds
  srand(time(NULL));
  srand48(time(NULL));
  //get all of the tensegrity structure's cables
  const std::vector<tgBasicActuator*> actuators = subject.getAllActuators();
  for (size_t i = 0; i < actuators.size(); ++i)
    {
      tgBasicActuator * const pActuator = actuators[i];
      assert(pActuator != NULL);
      //instantiate controllers for each cable
      tgBasicController* m_lenController = new tgBasicController(pActuator, m_length);
      //add controller to vector
      m_controllers.push_back(m_lenController);
      //generate random restlength
      double rand_max = 12;
      double rand_min = 2;
      double gen_len = drand48()*(rand_max-rand_min) + rand_min;
      rand_lengths.push_back(gen_len); 
    }
  //std::cout << rand_lengths.size();

  //define random indices to select 1st, 2nd and 3rd actuators at random
  //2nd and 3rd indices are placed in a while loop that prevents the
  //indices from repeating
  /*
    t1= rand() %24;
    t2= rand() %24;
    t3= rand() %24;
    while (t1==t2) {
    t2= rand() %24;
    }
    while (t1==t3 || t2==t3) {
    t3= rand() %24;
    }
  */
}

void LengthController::onStep(Model& subject, double dt)
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
