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
 * @file LengthController12BarOctahedron.cpp
 * @brief Implementation of LengthController12BarOctahedron.h
 * @author Hannah Petersson, based on files from Drew Sabelhaus and Mallory Daly
 * $Id$
 */

/* This length controller does... TBD
 * 
 * 
 */

// This module
#include "LengthController12BarOctahedron.h"

// This application
#include "yamlbuilder/TensegrityModel.h"

// This library
#include "core/tgBasicActuator.h" // All controllers are based on this
#include "core/tgSpringCableActuator.h" // OBS OBS is this for a system with springs?
#include "core/tgString.h" // Combining strings with ints for naming structures
#include "core/tgTags.h" // The ability to tag models and controllers
#include "sensors/tgDataObserver.h" // For viewing the data

// The C++ standard library 
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "helpers/FileHelpers.h"

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
LengthController12BarOctahedron::LengthController12BarOctahedron(double startTime, 
							double minLength, 
							double rate,
							 std::vector<std::string> tagsToControl) :
  m_startTime(startTime),
  m_minLength(minLength),
  m_rate(rate),
  m_tagsToControl(tagsToControl),
  m_timePassed(0.0)
{
  // Start time must be greater than or equal to zero
  if ( m_startTime < 0.0 ) {
    throw std::invalid_argument("Start time must be greater than or equal to zero.");
  }
  // Min length must be between 0 and 1 (it is a percentage) 
  else if( m_minLength > 1) {
    throw std::invalid_argument("min_Length is a percent, must be less than 1. (100%)");
  }
  else if( m_minLength < 0.0 ) {
    throw std::invalid_argument("min_Length is a percent, must be greater than 0.");
  }
  // Rate must be greater than zero
  else if( rate < 0.0 ) {
    throw std::invalid_argument("Rate cannot be negative.");
  }
  // Should there be checks made on the tags?
}

/**
 * The initializeActuators method is called in onSetup to put pointers to 
 * specific actuators in the cablesWithTags array, as well as store the initial
 * rest lengths in the initialRL map.
 */
void LengthController12BarOctahedron::initializeActuators(TensegrityModel& subject, std::string tag) {

  //DEBUGGING
  std::cout << "Finding cables with the tag: " << tag << std::endl;
  
  // Pick out the actuators with the specified tag
  std::vector<tgBasicActuator*> foundActuators = subject.find<tgBasicActuator>(tag);
  std::cout << "The following cables were found and will be controlled: " << std::endl;

  // If no actuators are found
  if (foundActuators.size() == 0){
    std::cout << "No actuators found." << std::endl;
  }

  // Iterate through array and output strings to command line
  for (std::size_t i = 0; i < foundActuators.size(); i++ ) {
    //std::cout << foundActuators[i] -> getTags() << std::endl; // This just prints 'cable'
    // Add the rest length of the actuator at this time to the list of all initial rest lengths
    initialRL[foundActuators[i]->getTags()] = foundActuators[i]->getRestLength();
    // DEBUGGING
    std::cout << "Cable " << i << " with rest length " << initialRL[foundActuators[i]->getTags()] << " at t = 0." << std::endl;
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
void LengthController12BarOctahedron::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up the LengthController12BarOctahedron controller." << std::endl;
 // std::cout << "Finding cables with tags: " << m_tagsToControl << std::endl;
  cablesWithTags = {};
 
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for( it = m_tagsToControl.begin(); it < m_tagsToControl.end(); it++ ) {
    // Call the helper for this tag
    initializeActuators(subject, *it);
  }   
  
  // Initialize flags 
  m_retract = 1; // Begin in tetract mode 
  m_finished = 0; // True when finished retracting and returning all cables 
  
  // Initialize cable index
  m_cable_index = 0;
  
  // Output that controller setup is complete
  std::cout << "Finished setting up the controller." << std::endl;
}

/**
 * Changes the cables' lengths at some specified timestep.
 * @param[in] subject - the TensegrityModel that is being controlled. Must
 * have a list of allMuscles populated
 * @param[in] dt, current timestep must be positive
 */
void LengthController12BarOctahedron::onStep(TensegrityModel& subject, double dt) {
  
  //std::cout << "Time passed: " << m_timePassed << std::endl;
 
 // First, increment the accumulator variable
  m_timePassed += dt;

  // If it is past the time to start the controller, start
  if( m_timePassed > m_startTime ) 
  {
    // Retract mode (retract each cable in sequence)
    if( m_retract == 1 ) {
      int i = m_cable_index; // Grab cable index. Unnecessary?
   //  std::cout << "Cable index: " << m_cable_index << std::endl;
//	 std::cout << "Entered retract mode for cable index: " << i << std::endl; // Debugging 

      double currRestLength = cablesWithTags[i]->getRestLength(); // Grab current rest length
  //    std::cout << "Current rest length is: " << currRestLength << std::endl;

      double minRestLength = initialRL[cablesWithTags[i]->getTags()] * m_minLength; // Calculate the minimum rest length for this cable
    //  std::cout << "Minimum rest length is: " << minRestLength << std::endl;

 // If the current rest length is greater than the desired minimum
      if( currRestLength > minRestLength ) {
        // Output a progress bar for the controller, to track when controll occurs
      //  std::cout << "Control occurs, cable index: " << i << std::endl;
        // Adjust rest length of the actuator
        double nextRestLength = currRestLength - m_rate*dt;

        // DEBUGGING
        //std::cout << "m_rate:  " << m_rate << std::endl;
        //std::cout << "dt: " << dt << std::endl;
        //std::cout << "Next rest length: " << nextRestLength << std::endl;
       
        cablesWithTags[i]->setControlInput(nextRestLength, dt); 
      }
      else {
         // Cable has been retracted to min length, go to next cable
        m_cable_index = m_cable_index + 1;
        std::cout << "Cable finished retracting, go to cable with index: " << m_cable_index << std::endl;
        // If the cable index is equal to the number of cables, all cables have been retracted
        // Move to return state
        if( m_cable_index == cablesWithTags.size() ) {
          m_cable_index = 0;
          m_retract = 0;
        }
      }
    }

  // Return state
  else if ( m_finished == 0 ) {
    //std::cout << "Made it to return state." << std::endl;
    int i = m_cable_index; // Grab cable index 
    double currRestLength = cablesWithTags[i]->getRestLength(); // Grab current rest length
    double initialRestLength = initialRL[cablesWithTags[i]->getTags()]; // Calculate initial rest length for this cable
    // If the current rest length is below initial rest length
    if( currRestLength < initialRestLength ) {
      // Output a progress bar for the controller to track when control occurs
      //std::cout << "Control occured. Cable index: " << i << std::endl;
      double nextRestLength = currRestLength = m_rate*dt; // Adjust rest length of the actuator 
      // DEBUGGING
      //std::cout << "Next rest length: " << nextRestLength << std::endl;
      cablesWithTags[i]->setControlInput(nextRestLength, dt);
    } 
    else {
      // Cable has been retracted to min length, go to next cable
      m_cable_index ++; // OBS OBS OBS
      std::cout << "Cable index: " << m_cable_index << std::endl;
      // If the cable index is equal to the number of cables, all cables have been retracted. Move to return state. 
      if( m_cable_index == cablesWithTags.size() ) { 
        m_finished = 1;
      }
    } 
  }
}
}
