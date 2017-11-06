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
 * @file LaikaWalkingController.cpp
 * @brief Implementation of LaikaWalkingController.
 * @author Edward Zhu, Drew Sabelhaus, Lara Janse van Vuuren
 * $Id$
 */

// This module
#include "LaikaWalkingController.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
// #include "LaikaWalkingModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"

//#include "sensors/tgDataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "helpers/FileHelpers.h"
#include <stdexcept>
#include <sstream>

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
LaikaWalkingController::LaikaWalkingController(double startTime,
						     double minLength,
						     double rate) :
  m_startTime(startTime),
  m_minLength(minLength),
  m_rate(rate),
  m_timePassed(0.0)
{
  // start time must be greater than or equal to zero
  if( m_startTime < 0.0 ) {
    throw std::invalid_argument("Start time must be greater than or equal to zero.");
  }
  // min length must be between 1 and 0
  else if( m_minLength > 1 ) {
    throw std::invalid_argument("minLength is a percent, must be less than 1. (100%)");
  }
  else if( m_minLength < 0.0) {
    throw std::invalid_argument("minLength is a percent, must be greater than 0.");
  }
  // rate must be greater than zero
  else if( rate < 0.0 ) {
    throw std::invalid_argument("Rate cannot be negative.");
  }
  // @TODO: what checks to make on tags?
}

/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void LaikaWalkingController::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up the LaikaWalking controller." << std::endl;

	actuatorTags.push_back("HF");
  actuatorTags.push_back("HR");
  actuatorTags.push_back("HL");
  actuatorTags.push_back("HB");
  actuatorTags.push_back("SFR");
  actuatorTags.push_back("SRL");
  actuatorTags.push_back("SBF");
  actuatorTags.push_back("SBL");

	std::cout << "Using actuators ";
	for (int i = 0; i < actuatorTags.size(); i++) {
		if (i == actuatorTags.size()-1) {
			std::cout << actuatorTags[i] << std::endl;
		}
		else {
			std::cout << actuatorTags[i] << ", ";
		}
	}
	m_allActuators = getAllActuators(subject, actuatorTags);
	std::cout << m_allActuators.size() << " actuators found" << std::endl;

  std::cout << "Finished setting up the controller." << std::endl;
}

void LaikaWalkingController::onStep(TensegrityModel& subject, double dt)
{

}

std::vector<tgBasicActuator*> LaikaWalkingController::getAllActuators(TensegrityModel& subject, std::vector<std::string> actuatorTags)
{
  std::vector<tgBasicActuator*> allActuators;

  for (int i = 0; i < actuatorTags.size(); i++) {
    // Sort through actuators to make sure the order is the same
    for (int j = 0; j < numVertebrae-1; j++) {
      std::ostringstream num1;
      std::ostringstream num2;
      num1 << j+1;
      num2 << j+2;

      std::string tag;
      tag = actuatorTags[i] + " t" + num1.str() + "/t" + num2.str();
      std::vector<tgBasicActuator*> actuator = subject.find<tgBasicActuator>(tag);
      // std::cout << tag << std::endl;

      // Make sure this list is not empty:
      if(actuator.empty()) {
        throw std::invalid_argument("No actuators found with " + actuatorTags[i] + ".");
      }
      // Now, we know that element 0 exists.
      // Confirm that it is not a null pointer.
      if(actuator[0] == NULL) {
        throw std::runtime_error("Pointer to the first actuator with " + actuatorTags[i] + " is NULL.");
      }
      allActuators.push_back(actuator[0]);
    }
  }

  return allActuators;
}
