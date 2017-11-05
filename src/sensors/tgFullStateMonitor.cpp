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
 * @file tgFullStateMonitor.cpp
 * @brief Contains the implementation of concrete class tgFullStateMonitor
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "tgFullStateMonitor.h"
// This application
#include "tgSensor.h"
// The C++ Standard Library
#include <stdexcept>
#include <cassert>
#include <iostream>
#include <vector> // for managing descendants of tgSenseables, and storing states.
//#include <sstream> // for converting a size_t to a string.


/**
 * The constructor does nothing. It's onSetup that should take the first reading
 * of the states.
 */
tgFullStateMonitor::tgFullStateMonitor()
{
  // Postcondition  
  assert(invariant());
}

/**
 * The destructor should not have to do anything.
 * The parent class
 * handles deletion of the sensors and sensor infos.
 */
tgFullStateMonitor::~tgFullStateMonitor()
{
}

/**
 * Setup will do two things:
 * (1) create the sensors based on the sensor infos that have been added and 
 *     the senseable objects that have also been added,
 * (3) Read in the sensor data to the currentState vector, once.
 */
void tgFullStateMonitor::setup()
{
  // Call the parent's setup method, which creates the sensors.
  tgDataManager::setup();
  // Now, m_sensors should be populated! This is (1) above.

  std::cout << "Setting up the tgFullStateMonitor." << std::endl;
  // (2) Take the first sensor reading. The helper function automatically writes
  // to the currentState vector.
  getCurrentStateHelper();

  /*
  // Iterate. For each sensor, output its header.
  // Prepend each label with the sensor number, which we choose to be the index in
  // the vector of sensors. NOTE that this means the sensors vector CANNOT
  // BE CHANGED, otherwise the data will not be aligned properly.
  for (std::size_t i=0; i < m_sensors.size(); i++) {
    // Get the vector of sensor data headings from this sensor
    std::vector<std::string> headings = m_sensors[i]->getSensorDataHeadings();
    // Iterate and output each heading
    for (std::size_t j=0; j < headings.size(); j++) {
      // Prepend with the sensor number and an underscore.
      // Also, end with a comma, since this is a comma-separated-value log file.
      tgOutput << i << "_" << headings[j] << ",";
    }
  }
  */
  
  // Postcondition
  assert(invariant());
}

/**
 * The parent's teardown method handles the sensors and sensor infos.
 */
void tgFullStateMonitor::teardown()
{
  // Call the parent's teardown method! This is important!
  tgDataManager::teardown();
  // Postcondition
  assert(invariant());
}

/**
 * The step method is where data is actually collected!
 * Here, the helper function is called. Real simple. Just update currentState.
 */
void tgFullStateMonitor::step(double dt) 
{
  if (dt <= 0.0)
  {
    throw std::invalid_argument("dt is not positive");
  }
  else
  {
    getCurrentStateHelper();
    /*
    // For the timestamp: first, add dt to the total time
    m_totalTime += dt;
    // also, add to the current time between sensor readings.
    m_updateTime += dt;
    // Then, if enough time has elapsed between the previous sensor reading,
    if (m_updateTime >= m_timeInterval) {
      // Open the log file for writing, appending and not overwriting.
      tgOutput.open(m_fileName.c_str(), std::ios::app);
      // Then output the time.
      tgOutput << m_totalTime << ",";
      // Collect the data and output it to the file!
      for (size_t i=0; i < m_sensors.size(); i++) {
	// Get the vector of sensor data from this sensor
	std::vector<std::string> sensordata = m_sensors[i]->getSensorData();
	// Iterate and output each data sample
	for (std::size_t j=0; j < sensordata.size(); j++) {
	  // Include a comma, since this is a comma-separated-value log file.
	  tgOutput << sensordata[j] << ",";
	}
      }
      tgOutput << std::endl;
      // Close the output, to be re-opened next step.
      tgOutput.close();
      // Now that the sensors have been read, reset the counter.
      m_updateTime = 0.0;
    }
    */
  }

  // Postcondition
  assert(invariant());
}

/**
 * The getCurrentStateHelper method is where all the "good stuff" happens.
 * Query all the sensors, assemble a big vector based on which sensors we want to
 * read from, then assign this vector to currentState.
 */
void tgFullStateMonitor::getCurrentStateHelper()
{
  
}

/**
 * The toString method for tgFullStateMonitor should have some specific information
 * about (for example) the log file...
 */
std::string tgFullStateMonitor::toString() const
{
  std::string p = "  ";  
  std::ostringstream os;
  os << tgDataManager::toString()
     << "This tgDataManager is a tgFullStateMonitor. " << std::endl;

  return os.str();
}

std::ostream&
operator<<(std::ostream& os, const tgFullStateMonitor& obj)
{
    os << obj.toString() << std::endl;
    return os;
}
