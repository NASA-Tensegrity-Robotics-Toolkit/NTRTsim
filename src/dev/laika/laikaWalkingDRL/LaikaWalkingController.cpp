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
#include "core/tgRod.h"
#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

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

  m_allActuators.clear();
  m_allControllers.clear();

  std::vector<std::string> actuatorTags;
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
	// std::cout << "Initial rest lengths: ";
	std::vector<double> desRL;
	for (int i = 0; i < m_allActuators.size(); i++) {
		actCableRL.push_back(m_allActuators[i]->getRestLength());
		desRL.push_back(actCableRL[i]);
		// if (i == m_allActuators.size()-1) {
		// 	std::cout << actCableRL[i] << std::endl;
		// }
		// else {
		// 	std::cout << actCableRL[i] << ",";
		// }
    if(actCableRL[i] == NULL) {
      throw std::runtime_error("Pointer to the first actuator with  is NULL.");
    }
		tgBasicController* m_lenController = new tgBasicController(m_allActuators[i], actCableRL[i]);
		m_allControllers.push_back(m_lenController);
	}

	// Update target rest lengths
	updateRestLengths(desRL);

	// Get pointers to the rigid bodies
	std::vector<std::string> shoulderTag;
	shoulderTag.push_back("shouldersBack");

	std::vector<std::string> hipTag;
	hipTag.push_back("hipsBack");

	std::vector<std::string> legTags;
	legTags.push_back("legBoxFrontLeft");
	legTags.push_back("legBoxFrontRight");
	legTags.push_back("legBoxBackLeft");
	legTags.push_back("legBoxBackRight");

	shoulderBody = getRigidBodies(subject, shoulderTag)[0];
	hipBody = getRigidBodies(subject, hipTag)[0];
	legBodies = getRigidBodies(subject, legTags);

	// Define initial torques
	double initialTorqueFL = 0.0;
	double initialTorqueFR = 0.0;
	double initialTorqueBL = -0.0;
	double initialTorqueBR = -0.0;

	std::vector<double> initialTorques;
	initialTorques.push_back(initialTorqueFL);
	initialTorques.push_back(initialTorqueFR);
	initialTorques.push_back(initialTorqueBL);
	initialTorques.push_back(initialTorqueBR);

	// Update initial torques
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

  // for (int i = 0; i < desCableRL.size(); i++) {
  //   if (i == desCableRL.size()-1) {
  //     std::cout << desCableRL[i] << std::endl;
  //   }
  //   else{
  //     std::cout << desCableRL[i] << ",";
  //   }
  // }
  // for (int i = 0; i < legTorques.size(); i++) {
  //   std::cout << legTorques[i].x() << "," << legTorques[i].y() << "," << legTorques[i].z() << std::endl;
  // }
	setRestLengths(dt);
	setTorques(dt);
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

std::vector<btRigidBody*> LaikaWalkingController::getRigidBodies(TensegrityModel& subject, std::vector<std::string> tags) {
	std::vector<btRigidBody*> rigidBodies;
	for (int i = 0; i < tags.size(); i++) {
		std::vector<tgBaseRigid*> rigids = subject.find<tgBaseRigid>(tags[i]);
		// std::cout << rigids.size() << std::endl;
		if( rigids.empty() ) {
	    throw std::invalid_argument("No rods found with " + tags[i] + ".");
	  }
	  // Now, we know that element 0 exists.
	  // Confirm that it is not a null pointer.
	  if( rigids[0] == NULL) {
	    throw std::runtime_error("Pointer to the first rod with " + tags[i] + " is NULL.");
	  }
		btRigidBody* rigidBody = rigids[0]->getPRigidBody();
		rigidBodies.push_back(rigidBody);
	}

	return rigidBodies;
}

void LaikaWalkingController::updateRestLengths(std::vector<double> controlRL) {
	if (controlRL.size() != cable_action_dim) {
		throw std::runtime_error("Cable action dimension mismatch");
	}
	desCableRL.assign(controlRL.begin(), controlRL.end());
}

void LaikaWalkingController::updateRestLengthsDiscrete(std::vector<double> controlRL, double targetVel, double dt) {
	if (controlRL.size() != cable_action_dim) {
		throw std::runtime_error("Cable action dimension mismatch");
	}
  for (int i = 0; i < controlRL.size(); i++) {
    if (controlRL[i] == -1) {
      desCableRL.push_back(m_allActuators[i]->getRestLength()-targetVel*dt);
    }
    else if (controlRL[i] == 1) {
      desCableRL.push_back(m_allActuators[i]->getRestLength()+targetVel*dt);
    }
    else if (controlRL[i] == 0) {
      desCableRL.push_back(m_allActuators[i]->getRestLength());
    }
    else {
      throw std::runtime_error("Unrecognized cable input");
    }
  }
}

void LaikaWalkingController::updateTorques(std::vector<double> controlTorques) {
	if (controlTorques.size() != leg_action_dim) {
		throw std::runtime_error("Leg action dimension mismatch");
	}

  std::vector<btVector3> tmp;
  for (int i = 0; i < controlTorques.size(); i++) {
    tmp.push_back(btVector3(0.0,0.0,controlTorques[i]));
  }
  legTorques.assign(tmp.begin(), tmp.end());
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
	// Order is FL, FR, BL, BR
	for (int i = 0; i < legTorques.size(); i++) {
		// std::cout << legTorques[i].x() << "," << legTorques[i].y() << "," << legTorques[i].z() << std::endl;
		legBodies[i]->applyTorqueImpulse(legTorques[i]);
		if (i == 0 || i == 1) { // Front legs
			shoulderBody->applyTorqueImpulse(-legTorques[i]);
		}
		else { // Back legs
			hipBody->applyTorqueImpulse(-legTorques[i]);
		}
	}
}
