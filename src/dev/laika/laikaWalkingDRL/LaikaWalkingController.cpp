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

using namespace boost::numeric::ublas;

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
LaikaWalkingController::LaikaWalkingController(bool train, double target_velocity)
{
m_train = train;
m_target_velocity = target_velocity;
}

/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void LaikaWalkingController::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up the LaikaWalking controller." << std::endl;

  worldTime = 0;

  // Set number of vertebrae and legs
  num_vertebrae = 5;
  num_legs = 4;
  std::cout << "Number of vertebrae: " << num_vertebrae << ", number of legs: " << num_legs << std::endl;

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
			std::cout << actuatorTags[i] << ". ";
		}
		else {
			std::cout << actuatorTags[i] << ", ";
		}
	}
	// Get actuators based on tags defined above
	m_allActuators = getAllActuators(subject, actuatorTags);
	std::cout << m_allActuators.size() << " actuators found" << std::endl;

  // Set action and state space dimension
  cable_action_dim = m_allActuators.size();
  leg_action_dim = 4;
  state_dim = int((num_vertebrae+num_legs)*12);
  std::cout << "Action space dimension: " << cable_action_dim + leg_action_dim << ", state space dimension: " << state_dim << std::endl;

	// Set up controllers
	std::vector<double> initialCableActions;
	for (int i = 0; i < m_allActuators.size(); i++) {
		initialCableActions.push_back(0.0);
    if(m_allActuators[i] == NULL) {
      throw std::runtime_error("Pointer to the first actuator with  is NULL.");
    }
		tgBasicController* m_lenController = new tgBasicController(m_allActuators[i], m_allActuators[i]->getRestLength());
		m_allControllers.push_back(m_lenController);
	}

	// Update target rest lengths
	updateRestLengthsDiscrete(initialCableActions,12.0,0.002);

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

  if (!m_train) {
    int in_dim = 176;
    int out_dim = 140;
    int hid_dim = 500;
    int n_layers = 2;
    bool transpose = true;
    dyn_nn.setNNParams(in_dim, out_dim, hid_dim, n_layers, transpose);

    dyn_nn.setLayerWeights(0, "fit_0_layer_0.csv");
    dyn_nn.setLayerWeights(1, "fit_0_layer_1.csv");
    dyn_nn.setLayerWeights(2, "fit_0_layer_out.csv");

    dyn_nn.setInputNormalization("fit_0_in_mean.csv", "fit_0_in_std.csv");
    dyn_nn.setOuputNormalization("fit_0_out_mean.csv", "fit_0_out_std.csv");

    int num_paths = 10;
    int horizon = 5;
    vector<double> action_ulim(cable_action_dim+leg_action_dim);
    vector<double> action_llim(cable_action_dim+leg_action_dim);
    action_ulim <<= 1, 1, 1, 1, 1, 1, 1, 1,
                    1, 1, 1, 1, 1, 1, 1, 1,
                    1, 1, 1, 1, 1, 1, 1, 1,
                    1, 1, 1, 1, 1, 1, 1, 1,
                    5, 5, 5, 5;
    action_llim = -action_ulim;
    controller.setMPCParams(horizon, num_paths, cable_action_dim, leg_action_dim);
    controller.attachDynamicsModel(&dyn_nn);
    controller.setInputLims(action_ulim, action_llim);
  }

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

  if (!m_train) {
    vector<double> body_states(getLaikaWalkingModelStates(subject));
    vector<double> rl_states(getCurrRestLengths());
    vector<double> states(state_dim+cable_action_dim);
    states <<= body_states, rl_states;
    // vector<double> actions(getCurrActions());
    // vector<double> next_states_pred(dyn_nn.getNNDynOutput(states, actions));
    // std::cout << states(0) << "," << next_states_pred(0) << std::endl;

    vector<double> actions(controller.getAction(states));
    std::vector<double> cable_cmd;
    std::vector<double> torque_cmd;
    for (int i = 0; i < actions.size(); i++) {
      if (i < cable_action_dim) {
        cable_cmd.push_back(actions(i));
      }
      else {
        torque_cmd.push_back(actions(i));
      }
    }
    updateRestLengthsDiscrete(cable_cmd, m_target_velocity, dt);
    updateTorques(torque_cmd);
  }

  setRestLengths(dt);
	setTorques(dt);
}

std::vector<tgBasicActuator*> LaikaWalkingController::getAllActuators(TensegrityModel& subject, std::vector<std::string> actuatorTags)
{
  std::vector<tgBasicActuator*> allActuators;

  for (int i = 0; i < actuatorTags.size(); i++) {
    // Sort through actuators to make sure the order is the same
    for (int j = 0; j < num_vertebrae-1; j++) {
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
        throw std::runtime_error("No actuators found with " + actuatorTags[i] + ".");
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
  desCableRL.clear();
  if (controlRL.size() != cable_action_dim) {
		throw std::runtime_error("Cable action dimension mismatch");
	}
	desCableRL.assign(controlRL.begin(), controlRL.end());
}

void LaikaWalkingController::updateRestLengthsDiscrete(std::vector<double> controlRL, double targetVel, double dt) {
  // Clear contents of vectors
  desCableRL.clear();
  currCableAction.clear();

  // Save cable rest lengths to be applied
  std::vector<double> tmp;
  if (controlRL.size() != cable_action_dim) {
		throw std::runtime_error("Cable action dimension mismatch");
	}
  for (int i = 0; i < controlRL.size(); i++) {
    if (controlRL[i] == -1) {
      tmp.push_back(m_allActuators[i]->getRestLength()-targetVel*dt);
    }
    else if (controlRL[i] == 1) {
      tmp.push_back(m_allActuators[i]->getRestLength()+targetVel*dt);
    }
    else if (controlRL[i] == 0) {
      tmp.push_back(m_allActuators[i]->getRestLength());
    }
    else {
      throw std::runtime_error("Unrecognized cable input");
    }
  }

  // Assign new contents
  desCableRL.assign(tmp.begin(), tmp.end());
  currCableAction.assign(controlRL.begin(), controlRL.end());
}

void LaikaWalkingController::updateTorques(std::vector<double> controlTorques) {
  // Clear contents of vectors
  legTorques.clear();
  currLegTorques.clear();

  if (controlTorques.size() != leg_action_dim) {
		throw std::runtime_error("Leg action dimension mismatch");
	}

  // Save torques to be applied in body frame
  std::vector<btVector3> tmp;
  for (int i = 0; i < controlTorques.size(); i++) {
    tmp.push_back(btVector3(0.0,0.0,controlTorques[i]));
  }

  // Assign new contents
  legTorques.assign(tmp.begin(), tmp.end());
  currLegTorques.assign(controlTorques.begin(), controlTorques.end());
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

vector<double> LaikaWalkingController::getLaikaWalkingModelStates(TensegrityModel& subject)
{
  // First, a list of all the tags we'll be picking out from the children.
  // In order, we want to do shoulder, vertebrae, hips, legs.
  // Pick out the box for each of the shoulders/hips and the boxes for the legs,
  // and the bottom rod of each of the three floating vertebrae.
  std::vector<std::string> laikaRigidBodyTags;
  laikaRigidBodyTags.push_back("shouldersBase");
  laikaRigidBodyTags.push_back("vertebraAbottomrod");
  laikaRigidBodyTags.push_back("vertebraBbottomrod");
  laikaRigidBodyTags.push_back("vertebraCbottomrod");
  laikaRigidBodyTags.push_back("hipsBase");
  laikaRigidBodyTags.push_back("legBoxBackLeft");
  laikaRigidBodyTags.push_back("legBoxBackRight");
  laikaRigidBodyTags.push_back("legBoxFrontLeft");
  laikaRigidBodyTags.push_back("legBoxFrontRight");

  if (int(laikaRigidBodyTags.size()*12) != state_dim) {
    throw std::invalid_argument("State dimension does not match expected dimension");
  }

  // We'll be putting the data here:
  vector<double> states(state_dim);
  // vector<double> states;

  // For each of the tags, do the following.
  // (1) get all the rigid bodies that have that tag
  // (2) confirm that there is exactly one element (one rigid)
  // (3) get the btRigidBody
  // (4) get the positions, orientations, velocities, and rot velocities
  // (5) append each of those to 'states'
  int state_counter = 0;
  for(int i=0; i < laikaRigidBodyTags.size(); i++) {

    // (1) get the rigid bodies with this tag
    std::vector<tgBaseRigid*> currentBodies =
      subject.find<tgBaseRigid>(laikaRigidBodyTags[i]);
    // Make sure this list is not empty:
    if( currentBodies.size() != 1 ) {
      throw std::invalid_argument("Wrong number of bodies with tag for states.");
    }
    // Now, we know that element 0 exists.
    // (2) Confirm that it is not a null pointer.
    if( currentBodies[0] == NULL) {
      throw std::runtime_error("Pointer to the first rigid body for states is NULL");
    }
    // (3)Get the single body.
    btRigidBody* currentBody = currentBodies[0]->getPRigidBody();

    // (4) In order, get and append positions, orient, vel, and rot vel
    btVector3 pos = currentBody->getCenterOfMassPosition();
    double yaw;
    double pitch;
    double roll;
    currentBody->getCenterOfMassTransform().getBasis().getEulerYPR(yaw, pitch, roll);
    btVector3 vel = currentBody->getLinearVelocity();
    btVector3 angularvel = currentBody->getAngularVelocity();

    // (5) put all this nice data into the 'states' vector.
    // Indexing into a btVector3 happens via the x, y, z methods. Elements 0, 1, 2.
    states.insert_element(state_counter,pos.x());
    state_counter++;
    states.insert_element(state_counter,pos.y());
    state_counter++;
    states.insert_element(state_counter,pos.z());
    state_counter++;
    states.insert_element(state_counter,yaw);
    state_counter++;
    states.insert_element(state_counter,pitch);
    state_counter++;
    states.insert_element(state_counter,roll);
    state_counter++;
    states.insert_element(state_counter,vel.x());
    state_counter++;
    states.insert_element(state_counter,vel.y());
    state_counter++;
    states.insert_element(state_counter,vel.z());
    state_counter++;
    states.insert_element(state_counter,angularvel.x());
    state_counter++;
    states.insert_element(state_counter,angularvel.y());
    state_counter++;
    states.insert_element(state_counter,angularvel.z());
    state_counter++;
    // if (i == laikaRigidBodyTags.size()-1) {
    //   std::cout << angularvel.y() << std::endl;
    // }
  }
  // std::cout << states(state_dim-2) << std::endl;
  return states;
}

vector<double> LaikaWalkingController::getCurrRestLengths()
{
  vector<double> rest_lengths(cable_action_dim);
  for (int i = 0; i < cable_action_dim; i++) {
    rest_lengths.insert_element(i,m_allActuators[i]->getRestLength());
  }
  return rest_lengths;
}

vector<double> LaikaWalkingController::getCurrActions()
{
  vector<double> actions(cable_action_dim+leg_action_dim);
  for (int i = 0; i < cable_action_dim+leg_action_dim; i++) {
    if (i < cable_action_dim) {
      actions.insert_element(i,currCableAction[i]);
    }
    else {
      actions.insert_element(i,currLegTorques[i-cable_action_dim]);
    }
  }
  return actions;
}
