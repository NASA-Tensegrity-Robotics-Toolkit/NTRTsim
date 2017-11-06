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
  // First, declare a new vector of doubles to which we'll append the data
  std::vector<double> updatedState;
  
  std::cout << "m_sensors.size is: " << m_sensors.size() << std::endl;
    
  // Then, loop through all the sensors. If any of the sensors are something
  // that this class "knows how to deal with," e.g. has a type that matches
  // a getSensorType() that's known.
  for (size_t i=0; i < m_sensors.size(); i++) {
    
    // Check if this sensor is something we're interested in.
    // First, let's just do compound rigid bodies.
    // TO-DO: make different sensor types OPTIONAL for this class,
    // e.g., maybe pass in some subset of which types to sense.
    // Alternatively: maybe use an "exclusion" parameter in the constructor,
    // that would do the following: if exclusion = 0, report back data on ALL
    // sensors. If exclusion = 1, do NOT report data from (example) tgRodSensors
    // that are also being sensed by a tgCompoundRigidSensor. That way, we won't
    // get back "double info" from the rods within compound rigids.

    if( m_sensors[i]->getSensorType() == "tgCompoundRigidSensor") {
      std::cout << "Will be taking data from sensor " << i << " which is a compound rigid sensor." << std::endl;
    }

    // ON 2017-11-05:
    // Major bugs, so this class is not used. Three main issues:

    // (1) Substructures in YAML are not being tagged the way I want.
    // For example, the rods within 't1' need to have 't1' as a tag, not just 'rod'.
    // Maybe, instead of (in addition to?) tagging compounded bodies according
    // to the hash I did, use the substructure name from YAML also.

    // (2) Structures are being presented out-of-order. This is kind of expected,
    // I guess, since it has to do with the randomness of the tags that are
    // assigned to the compound rigid bodies. However, we need consistent ordering
    // for the state vector that's being returned (the components need to refer
    // to the same rigid bodies from one run to the next!!!). So, we'd really need
    // to index by some externally defined name, not just alphabetize by compound
    // hash, since compound hash changes each time.

    // (3) BIGGEST BUG: not all compound bodies are sensed!!!! Or, maybe tags
    // aren't assigned to all compounded bodies. Example: run this with
    // AppLaikaWalkingDRL, and see that the number of sensors changes from 7, to 8,
    // to 9 seemingly randomly. There should always be 9 sensors.
    // However - this is probably more a problem with tgCompoundRigidSensor and its
    // friends, since the same "missing sensors" actually seems to happen with
    // tgDataLogger2 for the same App. Did Robel's code drop sensors like this?

    /*
    std::vector<std::string> sensordata = m_sensors[i]->getSensorData();
    // Iterate and output each data sample
    for (std::size_t j=0; j < sensordata.size(); j++) {
      // Include a comma, since this is a comma-separated-value log file.
      tgOutput << sensordata[j] << ",";
    }
    */
  }
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
