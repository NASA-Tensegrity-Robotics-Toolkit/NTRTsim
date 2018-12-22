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
 * @file InvKinTestController.cpp
 * @brief Implementation of InvKinTestController.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "InvKinTestController.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
// This library
//#include "core/tgRod.h"
#include "core/tgBasicActuator.h"
#include "core/tgBaseRigid.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"
#include "core/tgRod.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
// #include "BulletDynamics/Dynamics/btDynamicsWorld.h" // for hinge hack
// #include "BulletDynamics/ConstraintSolver/btHingeConstraint.h" // for hinge hack
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <fstream> // for the input CSV file
#include "helpers/FileHelpers.h" // what's this??
#include <stdexcept>
#include <stdlib.h> // for the atof, atoi functions

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
InvKinTestController::InvKinTestController(double startTime,
                  double holdTime, double period, std::string invkinCSVPath) :
  m_startTime(startTime),
  m_holdTime(holdTime),
  m_period(period),
  m_invkinCSVPath(invkinCSVPath),
  m_timePassed(0.0),
  m_timeSinceLastInput(0.0),
  m_inputIndex(0),
  m_numInputs(0)
{
  // start time must be greater than or equal to zero
  if( m_startTime < 0.0 ) {
    throw std::invalid_argument("Start time must be greater than or equal to zero.");
  }
  // hold time must be greater than or equal to zero.
  if( m_holdTime < 0.0 ) {
    throw std::invalid_argument("Hold time must be greater than or equal to zero.");
  }
  // the period needs to be nonzero and positive.
  if( m_period <= 0.0 ) {
    throw std::invalid_argument("Period must be positive.");
  }
  // The CSV file needs to not be null, or an empty string.
  if( m_invkinCSVPath.empty() ){
    throw std::invalid_argument("CSV path not passed in. Must provide a file with inv kin rest lengths.");
  }
  // If there's a tilde in the file name prefix, replace with the home directory
  // This copied from tgDataLogger2.
  if (m_invkinCSVPath.at(0) == '~') {
    // Get the $HOME environment variable
    std::string home = std::getenv("HOME");
    // Remove the tilde (the first element) from the string
    m_invkinCSVPath.erase(0,1);
    // Concatenate the home directory.
    m_invkinCSVPath = home + m_invkinCSVPath;
  }
  // @TODO: what asserts?
  // Note that unlike in C, we don't need to allocate the maps or vectors.
  // Declaring them creates empty ones for us.
}

/**
 * For this controller, the onSetup method does:
 * 
 */
void InvKinTestController::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up a InvKinTestController with CSV file: "
	    << m_invkinCSVPath << std::endl;
  // call the helpers to do the following: 
  // (a) get the list of tags and store the rest length inputs in their map
  // (b) assign the map of cables.
  assignCableInputMap();
  assignCableTagMap(subject);

  // Do some checks:
  // (1) neither result is empty
  // (2) there are no null pointers in either map
}

/**
 * The onStep method does one of the following things:
 * If between time zero and startTime: no change to rest length (the defaults from the YAML file are used.)
 * If between startTime and holdTime: apply the first rest length from the map ("let settle to initial pose")
 * If after holdTime: apply the rest length corresponding to the appropriate entry in the map
 */
void InvKinTestController::onStep(TensegrityModel& subject, double dt)
{
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, check which action to perform:
  if( m_timePassed > m_startTime ) {
    // Do *something*. Either the initial hold, or actually iterate through the list of inputs.
    // the hold time is in addition to start time.
    if(m_timePassed > (m_startTime + m_holdTime)) {
      // We now count the period for the actual control loop.
      // This is a bit off from a zero order hold implementation. What we do here is:
      // At 0 <= m_timeSinceLastInput < m_period, no change in the input. Iterate to next if m_timeSinceLastInput > period.
      // Since we've enforced that m_holdTime > 0, the "first" input is already being applied when
      // this loop runs for the first time, so we don't need to worry, just start counting towards the next input.

      // Increment the timing conter
      m_timeSinceLastInput += dt;
      // If appropriate, increment the index into the inputs
      // Two checks: 
      // (1) if the timer has fired for the input to change, 
      // (2) if we haven't fallen off the end of the list. Note that we correct for C's zero-indexing.
      //if( (m_timeSinceLastInput > m_period) && (m_inputIndex < (cableInputMap.size()-1)) ) {
      if( (m_timeSinceLastInput > m_period) && (m_inputIndex < (m_numInputs-1)) ) {
        // Advance to the next input in the list
        m_inputIndex++;
        // and reset the counter.
        m_timeSinceLastInput = 0.0;
      }
      // Finally, apply the resulting input.
      applyIthControlInput(m_inputIndex, dt);
    }
    else {
      // // Apply the first control input
      applyIthControlInput(0, dt);
    }
  }
  // else, do nothing.
}

// Implementation of the helper to onStep.
void InvKinTestController::applyIthControlInput(int i, double dt)
{
  //debugging
  std::cout << "At input number " << i << ", current and applied rest lengths are: " << std::endl;
  // Iterate over the map of inputs
  std::map<std::string, std::vector<double> >::iterator it = cableInputMap.begin();
  while(it != cableInputMap.end()) {
    // The tag here is
    std::string tag = it->first;
    // Get the pointer to the correct cable
    tgBasicActuator* currentCable = cableTagMap[tag];
    //debugging
    std::cout << currentCable->getRestLength() << ", ";
    // And the i-th input in the list of controls
    double currentRestLength = cableInputMap[tag][i];
    // Apply the rest length.
    // To-do: figure out why we need to call the version with dt here.
    // Some design decision was made long ago that may not make sense anymore...
    currentCable->setControlInput(currentRestLength, dt);
    //debugging
    std::cout << currentRestLength << " " << std::endl;
    // increment to the next tag (next cable).
    it++;
  }
  std::cout << std::endl;
}

// Implementations of the helpers.
void InvKinTestController::assignCableInputMap()
{
  // This function reads the CSV file, and parses the rows and columns into control inputs.
  // Open the CSV as a file input stream
  // This needs to be a c-style string for some reason.
  std::ifstream csvFile(m_invkinCSVPath.c_str());
  // If the file wasn't opened, throw an exception.
  if( !csvFile.is_open() ) {
    throw std::invalid_argument("Couldn't open the CSV file, check the path.");
  }
  // Now, loop over the rows.
  // Note that the getline function will return 0 when there are no more lines left, 
  // so the helper exist with an empty vector.
  // Initialize. Take the first set of elements:
  std::vector<std::string> row = InvKinTestController::getNextLineAndSplitIntoTokens(csvFile);
  // Our end condition will be when the next grab of a row is empty.
  // First, loop through until we can pick out the number of cables.
  // We know it's the 4th line. The first one is as above, so do three more:
  for(int i=0; i < 3; i++) {
    row = InvKinTestController::getNextLineAndSplitIntoTokens(csvFile);
  }
  //debugging
  // for(int i=0; i < row.size(); i++) {
  //   std::cout << row[i] << std::endl;
  // }
  //std::cout << row << std::endl;
  // We should now be on the 4th row. Pick out the number of cables. 
  // that's the 4th element, so counting from 0, it's index 3.
  int numCables = 0;
  try {
    // In C++98 we've got to to the int conversion this funny way
    //numCables = atoi(row[3]);
    std::istringstream(row[3]) >> numCables;
  }
  catch (const std::out_of_range& oor) {
    throw std::out_of_range("Inv Kin CSV file not formatted correctly. Exiting.");
  }
  //debugging
  std::cout << "Number of cables is " << numCables << std::endl;
  // Move down two more rows to get the tags.
  row = getNextLineAndSplitIntoTokens(csvFile);
  row = getNextLineAndSplitIntoTokens(csvFile);
  // Now at row 6, which should be the tags. We'll store them temporarily 
  // in an array, which will be our go-between for the map.
  // Vector assignment in C++ is a deep copy operation so this is fine.
  std::vector<std::string> cableTags = row;
  // Finally, we now work with the map. Get the first line of actual data:
  row = getNextLineAndSplitIntoTokens(csvFile);
  // for(int i=0; i < row.size(); i++) {
  //   std::cout << row[i] << std::endl;
  // }
  // Then iterate until the data is done.
  // What's returned is either an empty vector, or a vector with one element that's
  // an empty string. Catch either case.
  while(!row.empty() && !row[0].empty()) {
    // A quick check: if there is a newline at the end of CSV file,
    // a single-element vector with one empty string is returned here. Catch it.
    // if(row[0].empty()){
    //   throw std::out_of_range("CSV file has training newline. Remove it then re-run.");
    // }
    // At this row, the first numCables elements are for the cables.
    for(int i=0; i < numCables; i++) {
      // Get the key for this column. It's the cable tag.
      std::string key = cableTags[i];
      // Need to convert to float first, 
      double rl = 0;
      std::istringstream(row[i]) >> rl;
      // Add to the map's vector.
      // std::cout << "Pushing " << key << ", " << rl << std::endl;
      cableInputMap[key].push_back( rl );
    }
    // Advance one row.
    row = getNextLineAndSplitIntoTokens(csvFile);
  }
  // Here, query the map for the size of the first array of inputs.
  // That's the number of timesteps we expect for the control.
  std::map<std::string, std::vector<double> >::iterator it = cableInputMap.begin();
  // iterator now points to the first pair. "second" is the list of inputs for the first key, value pair.
  m_numInputs = it->second.size();
  //debugging
  std::cout << "InvKin CSV file read into memory." << std::endl;
}

// This helper picks out the pointers to the actuators.
void InvKinTestController::assignCableTagMap(TensegrityModel& subject) {
  // Iterate through the known cables that we need to control
  // It's important that this is called AFTER assignCableInputMap,
  // so that we can use the tags pulled out of the CSV file.
  std::map<std::string, std::vector<double> >::iterator it = cableInputMap.begin();
  while(it != cableInputMap.end()) {
    // The tag here is
    std::string tag = it->first;
    //debugging
    std::cout << "Finding cables with tag: " << tag << std::endl;
    // Pick out the actuators with the specified tag
    std::vector<tgBasicActuator*> foundActuators = subject.find<tgBasicActuator>(tag);
    // There should be exactly one cable here. If not throw an error.
    if( foundActuators.size() != 1 ) {
      throw std::runtime_error("Incorrect number of cables with tag " + tag );
    }
    // Store that one and only cable in the map.
    cableTagMap[tag] = foundActuators[0];
    // increment to the next cable tag
    it++;
  }
}

// The function to help parse the CSV file. Credit goes to StackOverflow's Martin York.
// https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
std::vector<std::string> InvKinTestController::getNextLineAndSplitIntoTokens(std::istream& str)
{
    std::vector<std::string>   result;
    std::string                line;
    std::getline(str,line);

    std::stringstream          lineStream(line);
    std::string                cell;

    while(std::getline(lineStream,cell, ','))
    {
        result.push_back(cell);
    }
    // This checks for a trailing comma with no data after it.
    if (!lineStream && cell.empty())
    {
        // If there was a trailing comma then add an empty element.
        result.push_back("");
    }
    return result;
}
 
