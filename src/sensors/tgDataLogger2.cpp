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
 * @file tgDataLogger2.cpp
 * @brief Contains the implementation of concrete class tgDataLogger2
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
//#include "tgDataManager.h"
#include "tgDataLogger2.h"
// This application
#include "tgSensor.h"
//#include "tgSensorInfo.h"
// The C++ Standard Library
//#include <stdio.h> // for sprintf
#include <stdexcept>
#include <cassert>
#include <iostream>
#include <vector> // for managing descendants of tgSenseables.

/**
 * The constructor for this class only assigns the filename prefix.
 * The actual filename is created in setup.
 * This makes it so that, upon reset, a new log file is opened (instead of
 * appending to the same one.)
 * Call the constructor of the parent class anyway, though it does nothing.
 */
tgDataLogger2::tgDataLogger2(std::string fileNamePrefix) :
  tgDataManager(),
  m_fileNamePrefix(fileNamePrefix)
{
  //DEBUGGING
  std::cout << "tgDataLogger2 constructor." << std::endl;
  // Postcondition
  assert(invariant());
}

/**
 * The destructor should not have to do anything.
 * Closing the log file is handled by teardown(), and the parent class
 * handles deletion of the sensors and sensor infos.
 */
tgDataLogger2::~tgDataLogger2()
{
  //DEBUGGING
  std::cout << "tgDataLogger2 destructor." << std::endl;
}

/**
 * Setup will do three things:
 * (1) create the full filename, based on the current time from the operating system,
 * (2) create the sensors based on the sensor infos that have been added and 
 *     the senseable objects that have also been added,
 * (3) opens the log file and writes a heading line (then closes the file.)
 */
void tgDataLogger2::setup()
{
  // Call the parent's setup method, which creates the sensors.
  tgDataManager::setup();
  //DEBUGGING
  std::cout << "tgDataLogger2 setup." << std::endl;

  // First, check that we're getting the tgSenseables that we want.
  // Each of the elements in m_senseables may have descendants.
  // Note that we do NOT check for duplicates, unlike the descendants management
  // in tgModel.
  std::vector< std::vector<tgSenseable*> > descendants;
  //DEBUGGING
  std::cout << "There are the following senseables that were directly attached to this tgDataLogger2: " << std::endl;
  for (size_t i=0; i < m_senseables.size(); i++) {
    std::cout << m_senseables[i]->getLabelForSensor() << std::endl;
    descendants.push_back( m_senseables[i]->getSenseableDescendants());
  }
  std::cout << "The descendants of each of these sense-ables are: " << std::endl;
  for (size_t i=0; i< descendants.size(); i++) {
    std::cout << "Senseable number: " << i << " has "
	      << descendants[i].size() << " descendants, and they are: "
	      << std::endl;
    for (size_t j=0; j < descendants[i].size(); j++) {
      std::cout << descendants[i][j]->getLabelForSensor() << std::endl;
    }
  }

  // Check how many sensor infos are attached.
  //DEBUGGING
  std::cout << "There are " << m_sensorInfos.size() << " sensor infos attached."
	    << std::endl;
  std::cout << "There are " << m_sensors.size() << " sensors attached."
	    << std::endl;
  
  // Postcondition
  assert(invariant());
}

/**
 * The parent's teardown method handles the sensors and sensor infos.
 */
void tgDataLogger2::teardown()
{
  // Call the parent's teardown method! This is important!
  tgDataManager::teardown();
  //DEBUGGING
  std::cout << "tgDataLogger2 teardown." << std::endl;
  // Postcondition
  assert(invariant());
}

/**
 * The step method is where data is actually collected!
 * This data logger will do two things here:
 * (1) iterate through all the sensors, collect their data, 
 * (2) write that line of data to the log file (then close the log again.)
 */
void tgDataLogger2::step(double dt) 
{
  //DEBUGGING
  //std::cout << "tgDataLogger2 step." << std::endl;
  if (dt <= 0.0)
  {
    throw std::invalid_argument("dt is not positive");
  }
  else
  {
    //DEBUGGING
    //std::cout << "tgDataLogger2 step." << std::endl;
    // TO-DO: collect the data.
  }

  // Postcondition
  assert(invariant());
}

/**
 * The toString method for tgDataLogger2 should have some specific information
 * about (for example) the log file...
 */
std::string tgDataLogger2::toString() const
{
  std::string p = "  ";  
  std::ostringstream os;
  // Note that we're using sprintf here to convert an int to a string.
  // TO-DO: fix this!
  os << "tgDataLogger2" << std::endl;
    //   << " with " << sprintf("%d",m_sensors.size()) << " sensors."<< std::endl;

  /*
    os << prefix << p << "Children:" << std::endl;
  for(std::size_t i = 0; i < m_children.size(); i++) {
    os << m_children[i]->toString(prefix + p) << std::endl;
  }
  os << prefix << p << "Tags: [" << getTags() << "]" << std::endl;
  os << prefix << ")";
  */
  return os.str();
}

std::ostream&
operator<<(std::ostream& os, const tgDataLogger2& obj)
{
    os << obj.toString() << std::endl;
    return os;
}
