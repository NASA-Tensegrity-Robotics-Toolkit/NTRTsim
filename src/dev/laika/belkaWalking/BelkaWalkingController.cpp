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
 * @file BelkaWalkingController.cpp
 * @brief Implementation of BelkaWalkingController.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "BelkaWalkingController.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
#include "BelkaWalkingModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"
#include "core/tgCast.h"

//#include "sensors/tgDataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "helpers/FileHelpers.h"
#include <stdexcept>

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
BelkaWalkingController::BelkaWalkingController() : m_timePassed(0.0)
{
  // onSetup should take care of any initialization: that way we can do teardown/restart properly.
}

/**
 * The initializeActuators method is call in onSetup to put pointers to 
 * specific actuators in the cablesWithTags array
 */
void BelkaWalkingController::initializeActuators(TensegrityModel& subject,
						    std::string tag) {
  //DEBUGGING
  std::cout << "Finding cables with the tag: " << tag << std::endl;
  // Pick out the actuators with the specified tag
  std::vector<tgBasicActuator*> foundActuators = subject.find<tgBasicActuator>(tag);
  std::cout << "The following cables were found and will be controlled: "
	    << std::endl;
  // Iterate through array and output strings to command line,
  // and record the cable's rest length.
  std::vector<double> rl;
  for (std::size_t i = 0; i < foundActuators.size(); i ++) {	
    std::cout << foundActuators[i]->getTags() << std::endl;
    rl.push_back(foundActuators[i]->getRestLength());
    // initialRL[foundActuators[i]->getTags()] = foundActuators[i]->getRestLength();
    //DEBUGGING:
    std::cout << "Cable rest length at t=0 is " << rl[i] << std::endl;
  }
  // Add to the map.
  cable_ptrs[tag] = foundActuators;
  // and now also the initial rest lengths.
  init_rest_lens[tag] = rl;
}

/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void BelkaWalkingController::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up the BelkaWalkingController." << std::endl;
  // Hard coding the tags here.
  // HF is the right horizontal set
  // HL is the bottom horizontal set maybe?
  // HR is the top horizontal set.
  // HB is the left horizontal set
  cableTags.push_back("HF");
  cableTags.push_back("HL");
  cableTags.push_back("HR");
  cableTags.push_back("HB");
  // Next four are the spine rotation.
  cableTags.push_back("SFR");
  cableTags.push_back("SFL");
  cableTags.push_back("SBF");
  cableTags.push_back("SBL");
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for( it = cableTags.begin(); it < cableTags.end(); it++ ) {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }
  // ***NOTE: leg hinges must be done elsewhere. At this point, they're not populated in the model yet.

  std::cout << "Finished setting up the controller." << std::endl;    
}

void BelkaWalkingController::onStep(TensegrityModel& subject, double dt)
{
  // Frustratingly, the leg hinges aren't populated in the model until AFTER the controller's onSetup method is called.
  // So, we've got to collect the pointers here.
  // First, cast the pointer to a BelkaWalkingModel from the superclass TensegrityModel.
  BelkaWalkingModel* subjectBelka = tgCast::cast<TensegrityModel, BelkaWalkingModel>(subject);
  if(legHinges.size() == 0){
    legHinges = subjectBelka->getLegHinges();
    // Enable the motors now.
    for(size_t i=0; i < legHinges.size(); i++){
      // some small dummy velocity to start with just so the body doesn't deactivate
      legHinges[i]->enableAngularMotor(true, 0.01, max_im);
    }
  }

  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, if it's passed the time to start the controller,
	// output a progress bar for the controller, to track when control occurs.
	// std::cout << "." << i;
	// Then, adjust the rest length of the actuator itself, according to
	// m_rate and dt.
	// double nextRestLength = currRestLength - m_rate * dt;
	//DEBUGGING
	//std::cout << "Next Rest Length: " << nextRestLength << std::endl;
  // std::cout << "onStep within BelkaWalkingController..." << std::endl;
	// cablesWithTags[i]->setControlInput(nextRestLength,dt);

  // For the leg motors: assume the first four entries in u_in are for the leg motors, in degrees.
  for(size_t i=0; i < legHinges.size(); i++){
    // legHinges[i]->setMotorTarget(u_in[i]*M_PI/180.0, dt);
    // In order to work with the keyboard callback, u_in is now stored in the model.
    legHinges[i]->setMotorTarget((subjectBelka->getU())[i] * (M_PI/180.0), dt);
  }

  // For the spine retraction percentages: first, the left/right cables, which are actually HF and HB
  std::vector<tgBasicActuator*> cbl_HF = cable_ptrs["HF"];
  std::vector<tgBasicActuator*> cbl_HB = cable_ptrs["HB"];
  std::vector<double> rl_HF = init_rest_lens["HF"];
  std::vector<double> rl_HB = init_rest_lens["HB"];
  for(size_t i=0; i < cbl_HF.size(); i++)
  {
    // Assuming horizontal left bend is positive cable retraction for HF,
    // and since u[4] is a percent retraction (example, =0.05 is 5% retraction),
    // the rest length now should be (1-u[4])*init_rest_len
    cbl_HF[i]->setControlInput((1 - subjectBelka->getU()[4])*rl_HF[i], dt);
  }
  for(size_t i=0; i < cbl_HB.size(); i++)
  {
    // Assuming horizontal left bend is cable extension for HB,
    // and since u[4] is a percent retraction,
    // the rest length now should be (1+u[4])*init_rest_len
    cbl_HB[i]->setControlInput((1 + subjectBelka->getU()[4])*rl_HB[i], dt);
  }

  // Then, the CCW/CW spine rotation, which involves two cables each.
  // The CWW (?) cables
  std::vector<tgBasicActuator*> cbl_SFR = cable_ptrs["SFR"];
  std::vector<tgBasicActuator*> cbl_SBF = cable_ptrs["SBF"];
  std::vector<double> rl_SFR = init_rest_lens["SFR"];
  std::vector<double> rl_SBF = init_rest_lens["SBF"];
  // The CW (?) cables
  std::vector<tgBasicActuator*> cbl_SFL = cable_ptrs["SFL"];
  std::vector<tgBasicActuator*> cbl_SBL = cable_ptrs["SBL"];
  std::vector<double> rl_SFL = init_rest_lens["SFL"];
  std::vector<double> rl_SBL = init_rest_lens["SBL"];

  for(size_t i=0; i < cbl_SFR.size(); i++)
  {
    // Assuming CCW is a positive cable retraction for SFR and SBF,
    // and since u[5] is a percent retraction (example, =0.05 is 5% retraction),
    // the rest length now should be (1-u[5])*init_rest_len
    // we can assume that both SFR and SBF are the same length vectors.
    cbl_SFR[i]->setControlInput((1 - subjectBelka->getU()[5])*rl_SFR[i], dt);
    cbl_SBF[i]->setControlInput((1 - subjectBelka->getU()[5])*rl_SBF[i], dt);
  }
  for(size_t i=0; i < cbl_SFL.size(); i++)
  {
    // Assuming CWW is a cable extension for SFL and SBL,
    // and since u[5] is a percent retraction,
    // the rest length now should be (1+u[5])*init_rest_len
    // we can assume that both SFL and SBL are the same length vectors.
    cbl_SFL[i]->setControlInput((1 + subjectBelka->getU()[5])*rl_SFL[i], dt);
    cbl_SBL[i]->setControlInput((1 + subjectBelka->getU()[5])*rl_SBL[i], dt);
  }
}
	
 
