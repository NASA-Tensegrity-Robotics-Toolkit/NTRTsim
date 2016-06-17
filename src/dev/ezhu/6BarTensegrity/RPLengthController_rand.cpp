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
#include <fstream>
#include <string>
// This module
#include "RPLengthController.h"
// This application
#include "../../../yamlbuilder/TensegrityModel.h"
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
}	

void RPLengthController::onSetup(TensegrityModel& subject)
{

  actuators = subject.getAllActuators();
  
}

void RPLengthController::onStep(TensegrityModel& subject, double dt)
{
  if (dt <= 0.0) {
    throw std::invalid_argument("dt is not positive");
  }
  else {
    for (int k = 0; k < (actuators.size()-1); k++) {
      //std::cout << k << ": " << actuators[k]->getTension() << ", ";
      std::cout << actuators[k] -> getTension() << ", ";
    }
    std::cout << actuators[actuators.size()-1] -> getTension() << std::endl;
  }
}

