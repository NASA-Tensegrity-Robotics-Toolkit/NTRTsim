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
 * @file LengthControllerYAML.cpp
 * @brief Implementation of LengthControllerYAML.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "LengthControllerYAML.h"
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

int ctr = 0;

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
LengthControllerYAML::LengthControllerYAML(double startTime,
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
void LengthControllerYAML::initializeActuators(TensegrityModel& subject,
					       std::string tag) {
  //DEBUGGING
  //std::cout << "Finding cables with the tag: " << tag << std::endl;
  // Pick out the actuators with the specified tag
  std::vector<tgBasicActuator*> foundActuators = subject.find<tgBasicActuator>(tag);
  //std::cout << "The following cables were found and will be controlled: "
	  //  << std::endl;
  //Iterate through array and output strings to command line
  for (std::size_t i = 0; i < foundActuators.size(); i ++) {	
    //std::cout << foundActuators[i]->getTags() << std::endl;
    // Also, add the rest length of the actuator at this time
    // to the list of all initial rest lengths.
    initialRL[foundActuators[i]->getTags()] = foundActuators[i]->getRestLength();
    //DEBUGGING:
    //std::cout << "Cable rest length at t=0 is "
	    //  << initialRL[foundActuators[i]->getTags()] << std::endl;
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
void LengthControllerYAML::onSetup(TensegrityModel& subject)
{
  //std::cout << "Setting up the LengthControllerYAML controller." << std::endl;
  //	    << "Finding cables with tags: " << m_tagsToControl
  //	    << std::endl;
  cablesWithTags = {};
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for( it = m_tagsToControl.begin(); it < m_tagsToControl.end(); it++ ) {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }
  //std::cout << "Finished setting up the controller." << std::endl;
  resetTimePassed();
  //getBallCOM(subject,31);    
}

void LengthControllerYAML::onStep(TensegrityModel& subject, double dt)
{
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, if it's passed the time to start the controller,
  if( m_timePassed > m_startTime ) {
    // For each cable, check if its rest length is past the minimum,
    // otherwise adjust its length according to m_rate and dt.
    //for (std::size_t i = 0; i < cablesWithTags.size()/3; i ++) {	
    for (std::size_t i = 0; i < 2; i ++) {  
      double currRestLength = cablesWithTags[i]->getRestLength();
      // Calculate the minimum rest length for this cable.
      // Remember that m_minLength is a percent.
      double minRestLength = initialRL[cablesWithTags[i]->getTags()] * m_minLength;
      // If the current rest length is still greater than the minimum,qqq
      if( currRestLength > minRestLength ) {
	// output a progress bar for the controller, to track when control occurs.
	//std::cout << "." << i;
	
  //std::cout << "Rest Length = " << currRestLength << std::endl;  
  
  // Then, adjust the rest length of the actuator itself, according to
	// m_rate and dt.
	double nextRestLength = currRestLength - m_rate * dt;
	//DEBUGGING
	//std::cout << "Next Rest Length: " << nextRestLength << std::endl;
	cablesWithTags[i]->setControlInput(nextRestLength,dt);
      }
      else if(ctr==0){
        std::cout << "Done at t=" << m_timePassed << std::endl;
        ctr++;
        getBallCOM(subject,32);
      }
    }   
  }
}

void LengthControllerYAML::resetTimePassed()
{
  ctr = 0;
  m_timePassed = 0;
}

void LengthControllerYAML::getBallCOM(TensegrityModel& subject, int color) 
{   
    //std::vector <tgRod*> rods = find<tgRod>("tgRodInfo");
    //assert(!rods.empty());

    btVector3 ballCenterOfMass(0, 0, 0);

    std::vector<tgRod*> foundRods = subject.find<tgRod>("prism_rod");
  
    double ballMass = 0.0; 
    for (std::size_t i = 0; i < foundRods.size(); i++) {
        const tgRod* const rod = foundRods[i];
        const double rodMass = rod->mass();
        const btVector3 rodCenterOfMass = rod->centerOfMass();
        //std::cout << i << ", " << rodCenterOfMass << ", ";
        ballCenterOfMass += rodCenterOfMass * rodMass;
        ballMass += rodMass;
    }
    //std::cout << std::endl;

    //assert(ballMass > 0.0);
    ballCenterOfMass /= ballMass;

    // Copy to the result std::vector
    std::vector<double> result(3);
    for (size_t i = 0; i < 3; ++i) 
    { 
      result[i] = ballCenterOfMass[i]; 
    }
    std::cout << "\e[1;" << color << "mX=" << result[0] << ", Y=" << result[1] << ", Z=" << result[2] << "\e[0m" << std::endl;
}