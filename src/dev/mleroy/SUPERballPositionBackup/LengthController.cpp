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
 * @file LengthController.cpp
 * @brief Implementation of class LengthController
 * @author Brian Cera
 * $Id$
 */

// This module
#include "LengthController.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>

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
}	

void LengthController::onSetup(threeBarModel& subject)
{

  m_controllers.clear(); //clear vector of controllers
  rand_lengths.clear(); //vector of randomized restlengths
  start_lengths.clear(); //vector of randomized restlengths
  
  //set seeds
  srand(time(NULL));
  srand48(time(NULL));
  
  //get all of the tensegrity structure's cables
  actuators = subject.getAllActuators();

  //Attach a tgBasicController to each actuator
  for (size_t i = 0; i < actuators.size(); ++i)
    {
      tgBasicActuator * const pActuator = actuators[i];
      assert(pActuator != NULL);
      //instantiate controllers for each cable
      tgBasicController* m_lenController = new tgBasicController(pActuator, m_length);
      //add controller to vector
      m_controllers.push_back(m_lenController);
      //generate random end restlength
      double start_length = actuators[i]->getStartLength();
      start_lengths.push_back(start_length);
      double rand_max = start_length*0.25; //maximum pos. deviation from start length
      double rand_min = -start_length*0.35; //maximum neg. deviation from start length
      double gen_len = drand48()*(rand_max-rand_min) + rand_min + start_length;
      rand_lengths.push_back(gen_len);
    }
}

void LengthController::onStep(threeBarModel& subject, double dt)
{
  if (dt <= 0.0) {
    throw std::invalid_argument("dt is not positive");
  }
  else {
    globalTime += dt;
    if(globalTime > 2){ //delay start of cable actuation
      if(toggle==0){ //print once when motors start moving
	cout << endl << "Activating Cable Motors (Randomized Lengths) -------------------------------------" << endl;
	toggle = 1;
      }
      if(toggle==1){
	toggle = 2;
	for(int i = 0; i<actuators.size(); i++){
	  m_controllers[i]->control(dt,rand_lengths[i]);
	  actuators[i]->moveMotors(dt);
	  if(actuators[i]->getRestLength()!=rand_lengths[i])
	    toggle = 1;
	}	
      }
      if(toggle==2){
	toggle = 1;
	for(int i = 0; i<actuators.size(); i++){
	  m_controllers[i]->control(dt,start_lengths[i]);
	  actuators[i]->moveMotors(dt);
	  if(actuators[i]->getRestLength()!=start_lengths[i])
	    toggle = 2;
	}	
      }
    }
  }
}

