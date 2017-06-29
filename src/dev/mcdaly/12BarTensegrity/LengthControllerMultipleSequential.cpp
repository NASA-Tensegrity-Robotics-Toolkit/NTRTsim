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
 * @file LengthControllerMultipleSequential.cpp
 * @brief Implementation of LengthControllerMultipleSequential.h
 * @author Drew Sabelhaus and Mallory Daly
 * $Id$
 */

/* This length controler retracts and returns one or more cables based on
 * the following inputs: controller start time, minimum cable length desired,
 * retraction rate, and cables to be controlled (based on tags).
 */

// This module
#include "LengthControllerMultipleSequential.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
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

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
LengthControllerMultipleSequential::LengthControllerMultipleSequential(double startTime,
					   double minLength,
					   double rate,
					   std::vector<std::string> tagsToControl) :
  m_startTime(startTime),
  m_minLength(minLength),
  m_rate(rate),
  m_tagsToControl(tagsToControl),
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
 * The initializeActuators method is call in onSetup to put pointers to 
 * specific actuators in the cablesWithTags array, as well as store the initial
 * rest lengths in the initialRL map.
 */
void LengthControllerMultipleSequential::initializeActuators(TensegrityModel& subject,
					       std::string tag) {
  //DEBUGGING
  std::cout << "Finding cables with the tag: " << tag << std::endl;
  // Pick out the actuators with the specified tag
  std::vector<tgBasicActuator*> foundActuators = subject.find<tgBasicActuator>(tag);
  std::cout << "The following cables were found and will be controlled: "
	    << std::endl;
  //Iterate through array and output strings to command line
  for (std::size_t i = 0; i < foundActuators.size(); i ++) {	
    std::cout << foundActuators[i]->getTags() << std::endl;
    // Also, add the rest length of the actuator at this time
    // to the list of all initial rest lengths.
    initialRL[foundActuators[i]->getTags()] = foundActuators[i]->getRestLength();
    //DEBUGGING:
    std::cout << "Cable rest length at t=0 is "
	      << initialRL[foundActuators[i]->getTags()] << std::endl;
  }
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
void LengthControllerMultipleSequential::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up the LengthControllerMultipleSequential controller." << std::endl;
  //	    << "Finding cables with tags: " << m_tagsToControl
  //	    << std::endl;
  cablesWithTags = {};
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for( it = m_tagsToControl.begin(); it < m_tagsToControl.end(); it++ ) {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }
  // Initialize flags
  m_retract = 1;  // Begin in retract mode
  m_finished = 0;  // True when finished retracting and returning all cables
  // Initialize cable index
  m_cable_index = 0;
  // Output that controller setup is complete
  std::cout << "Finished setting up the controller." << std::endl;    
}

void LengthControllerMultipleSequential::onStep(TensegrityModel& subject, double dt)
{
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, if it's passed the time to start the controller,
  if(m_timePassed > m_startTime) {   

    // Retract mode (retract each cable in sequence)
    if(m_retract == 1) {
      // Grab cable index
      int i = m_cable_index;
      // Grab current rest length
      double currRestLength = cablesWithTags[i]->getRestLength();
      // Calculate the minimum rest length for this cable
      double minRestLength = initialRL[cablesWithTags[i]->getTags()] * m_minLength;
      // If the current rest length is greater than the desired minimum,
      if(currRestLength > minRestLength) {
        // output a progress bar for the controller, to track when control occurs
        //std::cout << "." << i;
        // Then adjust the rest length of the actuator itself, according to
        // m_rate and dt
        double nextRestLength = currRestLength - m_rate * dt;
        //DEBUGGING
        //std::cout << "Next Rest Length: " << nextRestLength << std::endl;
        cablesWithTags[i]->setControlInput(nextRestLength,dt);
      }
      else {
        // Cable has been retracted to min length; now go to next cable
        m_cable_index += 1;
        std::cout << "Cable index: " << m_cable_index << std::endl;
        // If the cable index is equal to the number of cables, all cables have
        // been retracted (because of zero indexing). Move to return state.
        if(m_cable_index == cablesWithTags.size()) {
          m_cable_index = 0;
          m_retract = 0;

        }
      }
    }

    // Return state
    else if (m_finished == 0) {
      //std::cout << "Made it to return state." << std::endl; 
      // Grab cable index
      int i = m_cable_index;
      // Grab current rest length
      double currRestLength = cablesWithTags[i]->getRestLength();
      // Calculate the initial rest length for this cable
      double initialRestLength = initialRL[cablesWithTags[i]->getTags()];
      // If the current rest length is below initial rest length,
      if(currRestLength < initialRestLength) {
        // output a progress bar for the controller, to track when control occurs.
        //std::cout << "." << i;
        // Then adjust the rest length of the actuator itself, according to
        // m_rate and dt
        double nextRestLength = currRestLength + m_rate * dt;
        //DEBUGGING
        //std::cout << "Next Rest Length: " << nextRestLength << std::endl;
        cablesWithTags[i]->setControlInput(nextRestLength,dt);
      }
      else {
        // Cable has been retracted to min length; now go to next cable
        m_cable_index += 1;
        std::cout << "Cable index: " << m_cable_index << std::endl;
        // If the cable index is equal to the number of cables, all cables have
        // been retracted (because of zero indexing). Move to return state.
        if(m_cable_index == cablesWithTags.size()) {
          m_finished = 1;
        }
      }
    }
  }
}