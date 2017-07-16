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
 * @file LengthController12BarCube.cpp
 * @brief Implementation of LengthController12BarCube.h
 * @author Drew Sabelhaus and Mallory Daly
 * $Id$
 */

/* This length controller is meant specifically for creating the walking
 * scheme of the 12-bar cube. It retracts in sequence and then returns in 
 * sequence the first three cables in the actuated cable list. Then it
 * retracts and returns the next cable, and so on in sets of 3 and 1 for as
 * many cables are provided.
 */

// This module
#include "LengthController12BarCube.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"
#include "core/tgRod.h"

//#include "sensors/tgDataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "helpers/FileHelpers.h"
#include <stdexcept>
#include <fstream>

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
LengthController12BarCube::LengthController12BarCube(double startTime,
					   double minLength,
					   double rate,
             bool loop,
             std::vector<int> sequence,
					   std::vector<std::string> tagsToControl) :
  m_startTime(startTime),
  m_minLength(minLength),
  m_rate(rate),
  m_loop(loop),
  m_sequence(sequence),
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
  if (m_loop == true) {
    std::cout << "Looping infinitely." << std::endl;
  }
}

/**
 * The initializeActuators method is call in onSetup to put pointers to 
 * specific actuators in the cablesWithTags array, as well as store the initial
 * rest lengths in the initialRL map.
 */
void LengthController12BarCube::initializeActuators(TensegrityModel& subject,
					       std::string tag) {
  //DEBUGGING
  std::cout << "Finding cables with the tag \"" << tag << ".\"" << std::endl;
  // Pick out the actuators with the specified tag
  std::vector<tgBasicActuator*> foundActuators = subject.find<tgBasicActuator>(tag);
  // std::cout << "The following cables were found and will be controlled: "
	    // << std::endl;
  //Iterate through array and output strings to command line
  for (std::size_t i = 0; i < foundActuators.size(); i ++) {	
    // std::cout << foundActuators[i]->getTags() << std::endl;
    // Also, add the rest length of the actuator at this time
    // to the list of all initial rest lengths.
    initialRL[foundActuators[i]->getTags()] = foundActuators[i]->getRestLength();
    //DEBUGGING:
    // std::cout << "Cable rest length at t=0 is "
	   //    << initialRL[foundActuators[i]->getTags()] << std::endl;
  }
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  cablesWithTags.insert( cablesWithTags.end(), foundActuators.begin(),
			 foundActuators.end() );
}


void LengthController12BarCube::initializeRods(TensegrityModel& subject) {
  //DEBUGGING
  // Pick out the actuators with the specified tag
  std::vector<tgRod*> foundRods = subject.find<tgRod>("rod");

  // std::cout << "The following cables were found and will be controlled: "
      // << std::endl;
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  rodsToSave.insert( rodsToSave.end(), foundRods.begin(),
       foundRods.end() );

  for (std::size_t i = 0; i < foundRods.size(); i ++) {
    rodBodies.push_back(foundRods[i]->getPRigidBody());
  }
}

/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void LengthController12BarCube::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up the controller." << std::endl;
  //	    << "Finding cables with tags: " << m_tagsToControl
  //	    << std::endl;
  cablesWithTags = {};
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for( it = m_tagsToControl.begin(); it < m_tagsToControl.end(); it++ ) {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }
  initializeRods(subject);
  // Initialize flags
  retract = 1;  // Begin in retract mode
  finished = 0;  // True when finished retracting and returning all cables
  // Initialize cable index
  cable_idx = 0;
  // Grab number of sets
  num_sets = m_sequence.size()/4;
  current_set = 0;
  on_octagon = 1;
  // Output that controller setup is complete
  std::cout << "Finished setting up the controller." << std::endl; 
  std::cout << "----------------------------------------" << std::endl;     

  sim_out.open("cube_single_step.csv");
  sim_out << "time, ";
  for (std::size_t i = 0; i < rodBodies.size(); i++) {
      sim_out << "rod " << i << " x, " << "rod " << i << " y, " << "rod " << i << " z, " <<
                 "rod " << i << " axial_vec_x, " << "rod " << i << " axial_vec_y, " << "rod " << i << " axial_vec_z, ";
  }
  sim_out << std::endl;
  // std::cout << "Number of rod bodies: " << rodBodies.size() << std::endl;
  // sim_out << std::endl;

}

void LengthController12BarCube::onStep(TensegrityModel& subject, double dt)
{
  sim_out << m_timePassed << ", ";
  // Sim out results
  for (std::size_t i = 0; i < rodBodies.size(); i++) {
    sim_out << rodBodies[i]->getCenterOfMassPosition().x() << ", " << 
               rodBodies[i]->getCenterOfMassPosition().y() << ", " << 
               rodBodies[i]->getCenterOfMassPosition().z() << ", " <<
               (rodBodies[i]->getCenterOfMassTransform()*btVector3(0,1,0)).x() << ", " <<
               (rodBodies[i]->getCenterOfMassTransform()*btVector3(0,1,0)).y() << ", " <<
               (rodBodies[i]->getCenterOfMassTransform()*btVector3(0,1,0)).z() << ", ";               
  }
  sim_out << std::endl;
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, if it's passed the time to start the controller,
  if(m_timePassed > m_startTime) {

    // Retract mode (retract each cable in sequence)
    if(retract == 1 && finished == 0) {
      // Create index adjusted for current set
      int seq_idx = cable_idx + 4*current_set;
      int i = m_sequence[seq_idx];
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
        std::cout << "Retracted cable " << i << "." << std::endl;                
        // Cable has been retracted to min length; now go to next cable
        cable_idx += 1;
        // Check if I'm done retracting from the octagon face
        // if (cable_idx == 3 && on_octagon == 1) {
        if (cable_idx == 3 && on_octagon == 1) {          
          cable_idx = 0;
          retract = 0;
        }
        // Check if I'm done retracting from the triangle face
        else if (cable_idx == 4) {
          retract = 0;
          cable_idx = 3;
        }
      }
    }

    // Return state
    else if (finished == 0) {
      //std::cout << "Made it to return state." << std::endl; 
      // Create index adjusted for current set
      int seq_idx = cable_idx + 4*current_set;
      int i = m_sequence[seq_idx];
      // Grab current rest length
      double currRestLength = cablesWithTags[i]->getRestLength();
      // Calculate the initial rest length for this cable
      double initialRestLength = initialRL[cablesWithTags[i]->getTags()];
      // If the current rest length is below initial rest length,
      if (currRestLength < initialRestLength) {
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
        std::cout << "Returned cable " << i << "." << std::endl;
        // Cable has been retracted to min length; now go to next cable
        cable_idx += 1;
        // If the cable index is equal to the number of cables, all cables have
        // been retracted (because of zero indexing). Move to return state.
        if (cable_idx == 3) {
          on_octagon = 0;
          retract = 1;
        }
        else if (cable_idx == 4) {
          on_octagon = 1;
          retract = 1;
          current_set += 1;
          cable_idx = 0;
          if (current_set == num_sets) {
            if (m_loop == true) {
              current_set = 0;
            }
            else {
              finished = 1;
            }
          }
        std::cout << "----------------------------------------" << std::endl;          
        }
      }


    }
  }
}