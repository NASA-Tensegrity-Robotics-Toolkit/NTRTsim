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
#include "controllers/tgBasicController.h"
#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"
#include "LinearMath/btVector3.h"
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
LaikaWalkingController::LaikaWalkingController()
{

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
	// Get actuators based on tags defined above
	m_allActuators = getAllActuators(subject, actuatorTags);
	std::cout << m_allActuators.size() << " actuators found" << std::endl;

	// Set action space dimension
	cable_action_dim = m_allActuators.size();

	// Set up controllers
	std::cout << "Initial rest lengths: ";
	for (int i = 0; i < m_allActuators.size(); i++) {
		actCableRL.push_back(m_allActuators[i]->getRestLength());
		if (i == m_allActuators.size()-1) {
			std::cout << actCableRL[i] << std::endl;
		}
		else {
			std::cout << actCableRL[i] << ",";
		}
		tgBasicController* m_lenController = new tgBasicController(m_allActuators[i], actCableRL[i]);
		m_allControllers.push_back(m_lenController);
	}

	updateRestLengths(actCableRL);

	btVector3 initialTorqueFL(0.0, 0.0, 0.0);
	btVector3 initialTorqueFR(0.0, 0.0, 0.0);
	btVector3 initialTorqueBL(0.0, 0.0, 0.0);
	btVector3 initialTorqueBR(0.0, 0.0, 0.0);

	std::vector<btVector3> initialTorques;
	initialTorques.push_back(initialTorqueFL);
	initialTorques.push_back(initialTorqueFR);
	initialTorques.push_back(initialTorqueBL);
	initialTorques.push_back(initialTorqueBR);

	updateTorques(initialTorques);

  std::cout << "Finished setting up the controller." << std::endl;
}

void LaikaWalkingController::onStep(TensegrityModel& subject, double dt)
{
	if (dt <= 0.0) {
		throw std::invalid_argument("onStep: dt is not positive");
	}
	else {
		worldTime += dt;
	}

	setRestLengths(dt);
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

void LaikaWalkingController::updateRestLengths(std::vector<double> controlRL) {
	if (controlRL.size() != cable_action_dim) {
		throw std::runtime_error("Cable action dimension mismatch");
	}
	desCableRL.assign(controlRL.begin(), controlRL.end());
}

void LaikaWalkingController::updateTorques(std::vector<btVector3> controlTorques) {
	if (controlTorques.size() != leg_action_dim) {
		throw std::runtime_error("Leg action dimension mismatch");
	}
	legTorques.assign(controlTorques.begin(), controlTorques.end());
}

void LaikaWalkingController::setRestLengths(double dt) {
	// std::cout << "Desired rest length: ";
	for (int i = 0; i < m_allControllers.size(); i++) {
		// if (i == m_allControllers.size()-1) {
		// 	std::cout << desCableRL[i] << std::endl;
		// }
		// else {
		// 	std::cout << desCableRL[i] << ",";
		// }
		m_allControllers[i]->control(dt, desCableRL[i]);
		m_allActuators[i]->moveMotors(dt);
	}
}

void LaikaWalkingController::setTorques(double dt) {
	
}
