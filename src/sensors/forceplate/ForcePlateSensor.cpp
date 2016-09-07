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
 * @file ForcePlateSensor.cpp
 * @brief Implementation of a sensor for the ForcePlateModel.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "ForcePlateSensor.h"
// The model to sense
#include "ForcePlateModel.h"
// This library
#include "core/tgString.h"
#include "sensors/tgDataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

#include "helpers/FileHelpers.h"

/**
 * The constructor here assigns the two private variables, and
 * calls the constructor for the data observer.
 */
ForcePlateSensor::ForcePlateSensor(std::string path, double timeBetweenSamples):
  m_timeBetweenSamples(timeBetweenSamples),
  m_updateTime(0.0),
  m_dataObserver(path + "ForcePlateSensor_")
{
  // @TODO: check if path is valid??
}

/**
 * Setup only needs to call the setup method for the data observer.
 * This is the step that actually opens up a file for editing.
 */
void ForcePlateSensor::onSetup(ForcePlateModel& subject){
  m_dataObserver.onSetup(subject);
}

/**
 * The onStep method 'notifies' the data observer at intervals, controlled by
 * m_timeBetweenSamples. Every time that the observer is 'notified', it takes
 * a sample and saves it to the log file.
 */
void ForcePlateSensor::onStep(ForcePlateModel& subject, double dt) {
  // Check here: if the simulation ever outputs less than 0.0 seconds betwee
  // timesteps, something is very wrong.
  if (dt <= 0.0) {
    throw std::invalid_argument("dt is not positive");
  }
  else {
    // Else, increment m_updateTime, first:
    m_updateTime += dt;
    // Then, if the amount of time has reached the time in between samples,
    // call the data observer and reset the counter.
    if (m_updateTime >= m_timeBetweenSamples) {
      //DEBUGGING
      if( 1 ) {
	std::cout << "ForcePlateSensor notifying dataObserver, m_updateTime : "
		  << m_updateTime << std::endl
		  << "This force plate model has tags: "
		  << subject.getTags() << std::endl;
      }
      // Notify the observer (log the data)
      notifyStep(m_updateTime);
      m_dataObserver.onStep(subject, m_updateTime);
      // reset the counter
      m_updateTime = 0.0;      
    }
  }
}
