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
BelkaWalkingController::BelkaWalkingController(
						     std::vector<std::string> spineTags) :
  m_spineTags(spineTags),
  m_timePassed(0.0)
{
  // @TODO: what checks to make on tags?
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
  // //Iterate through array and output strings to command line
  // for (std::size_t i = 0; i < foundActuators.size(); i ++) {	
  //   std::cout << foundActuators[i]->getTags() << std::endl;
  //   // Also, add the rest length of the actuator at this time
  //   // to the list of all initial rest lengths.
  //   initialRL[foundActuators[i]->getTags()] = foundActuators[i]->getRestLength();
  //   //DEBUGGING:
  //   std::cout << "Cable rest length at t=0 is "
	//       << initialRL[foundActuators[i]->getTags()] << std::endl;
  // }
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  cablesWithTags.insert( cablesWithTags.end(), foundActuators.begin(),
			 foundActuators.end() );
}

/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void BelkaWalkingController::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up the BelkaWalkingController." << std::endl;
  //	    << "Finding cables with tags: " << m_tagsToControl
  //	    << std::endl;
  cablesWithTags = {};
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for( it = m_spineTags.begin(); it < m_spineTags.end(); it++ ) {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }
  // And get the hinges from the model.
  // First, cast the pointer to a BelkaWalkingModel from the superclass TensegrityModel.
  BelkaWalkingModel* subjectBelka = tgCast::cast<TensegrityModel, BelkaWalkingModel>(subject);
  legHinges = subjectBelka->getLegHinges();
  std::cout << "Finished setting up the controller." << std::endl;    
}

void BelkaWalkingController::onStep(TensegrityModel& subject, double dt)
{
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
	// cablesWithTags[i]->setControlInput(nextRestLength,dt);
}
	
 
