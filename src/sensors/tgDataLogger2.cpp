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
#include "tgDataLogger2.h"
// This application
#include "tgSensor.h"
// The C++ Standard Library
#include <stdexcept>
#include <cassert>
#include <iostream>
#include <vector> // for managing descendants of tgSenseables.
#include <time.h> // for the file name of the log file
#include <sstream> // for converting a size_t to a string.
#include <cstdlib> // for getenv, converting ~ to $HOME.

/**
 * The constructor for this class only assigns the filename prefix.
 * The actual filename is created in setup.
 * This makes it so that, upon reset, a new log file is opened (instead of
 * appending to the same one.)
 * Call the constructor of the parent class anyway, though it does nothing.
 */
tgDataLogger2::tgDataLogger2(std::string fileNamePrefix, double timeInterval) :
  tgDataManager(),
  m_fileNamePrefix(fileNamePrefix),
  m_timeInterval(timeInterval)
{
  // A quick check on the passed-in string: it must not be the empty
  // string. Must be a correct linux path.
  // TO-DO: a better check on this.
  if (m_fileNamePrefix == "") {
    throw std::invalid_argument("File name cannot be the empty string. Please pass in a path to a file that can be opened.");
  }
  // Check: a negative time interval doesn't make sense.
  if (m_timeInterval < 0.0 ) {
    throw std::invalid_argument("Time interval must be nonnegative. Negative time intervals do not make sense.");
  }

  // Additionally: we want to be able to expand the "~" string in fileNamePrefix
  // to be the full home directory of the current user.
  // It's a real pain to have to specify the complete directory structure
  // for these log file names.
  // Check if the first character of the string is a tilde:
  if (m_fileNamePrefix.at(0) == '~') {
    // Get the $HOME environment variable
    std::string home = std::getenv("HOME");
    // Remove the tilde (the first element) from the string
    m_fileNamePrefix.erase(0,1);
    // Concatenate the home directory.
    m_fileNamePrefix = home + m_fileNamePrefix;
  }
  
  // Postcondition  
  assert(invariant());
}

/**
 * The "BAD" constructor. 
 * Since people who use NTRT are apt to be new to C++, the errors that they'd get
 * when trying to create a tgDataLogger2 may be confusing. So, this constructor
 * lets the simulator compile, but then complains when it's called.
 * DO NOT USE THIS ONE: use the one with the string passed in!
 */
tgDataLogger2::tgDataLogger2()
{
  throw std::invalid_argument("Cannot create a tgDataLogger2 without a path to the log file! Please use the constructor that takes a string.");
}

/**
 * The destructor should not have to do anything.
 * Closing the log file is handled by teardown(), and the parent class
 * handles deletion of the sensors and sensor infos.
 */
tgDataLogger2::~tgDataLogger2()
{
  // TO-DO: should we double-check and close the tgOutput filestream here too?
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
  // Now, m_sensors should be populated! This is (2) above.

  // (1) Create the full filename of the log file.
  // Credit to Brian Tietz Mirletz, via the original tgDataObserver.
  // Adapted from: http://www.cplusplus.com/reference/clibrary/ctime/localtime/
  // Also http://www.cplusplus.com/forum/unices/2259/
  time_t rawtime;
  tm* currentTime;
  int fileTimeSize = 64;
  char fileTime [fileTimeSize];
  
  time (&rawtime);
  currentTime = localtime(&rawtime);
  strftime(fileTime, fileTimeSize, "%m%d%Y_%H%M%S", currentTime);
  // Result: fileTime is a string with the time information.
  m_fileName = m_fileNamePrefix + "_" + fileTime + ".txt";

  // DEBUGGING output:
  std::cout << "tgDataLogger2 will be saving data to the file: " << std::endl
	    << m_fileName << std::endl;

  // Attempt to open the log file
  tgOutput.open(m_fileName.c_str());
  if (!tgOutput.is_open()) {
    throw std::runtime_error("Log file could not be opened. Usually, this is because the directory you specified does not exist. Check for spelling errors.");
  }

  // Output a first line of the header.
  tgOutput << "tgDataLogger2 started logging at time " << fileTime << ", with "
	   << m_sensors.size() << " sensors on " << m_senseables.size()
	   << " senseable objects." << std::endl;

  // The first column of data will be "time", the m_totalTime since beginning
  // of the simulation.
  tgOutput << "time,";

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
  // End with a new line.
  tgOutput << std::endl;

  // Done! Close the output for now, will be re-opened during step.
  tgOutput.close();

  // Initialize/reset the values of the time variables.
  m_totalTime = 0.0;
  m_updateTime = 0.0;
  
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
  // Close the log file.
  tgOutput.close();
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
  if (dt <= 0.0)
  {
    throw std::invalid_argument("dt is not positive");
  }
  else
  {
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
  os << tgDataManager::toString()
     << "This tgDataManager is a tgDataLogger2. " << std::endl;

  return os.str();
}

std::ostream&
operator<<(std::ostream& os, const tgDataLogger2& obj)
{
    os << obj.toString() << std::endl;
    return os;
}
