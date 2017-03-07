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
 * @file tgSpringCableActuatorSensorInfo.cpp
 * @brief Contains the implementation of concrete class 
 *    tgSpringCableActuatorSensorInfo
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "tgSpringCableActuatorSensorInfo.h"
// Other includes from NTRTsim
#include "tgSpringCableActuatorSensor.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgSenseable.h"
#include "core/tgCast.h"
// Other includes from the C++ standard library
#include <stdexcept>

/**
 * Nothing to do in this constructor. A sensor info doesn't have any data.
 */
tgSpringCableActuatorSensorInfo::tgSpringCableActuatorSensorInfo()
{
}

/**
 * Similarly, empty destructor.
 */
tgSpringCableActuatorSensorInfo::~tgSpringCableActuatorSensorInfo()
{
}

/**
 * To check if a tgSenseable is a tgSpringCableActuator, we can use tgCast.
 */
bool tgSpringCableActuatorSensorInfo::isThisMySenseable(tgSenseable* pSenseable)
{
  // The following cast will return 0 if the senseable
  // is not a tgSpringCableActuator.
  tgSpringCableActuator* m_pRod =
    tgCast::cast<tgSenseable, tgSpringCableActuator>(pSenseable);
  if( m_pRod == 0 )
    return 0;
  else {
    return 1;
  }
}

/**
 * Create a sensor for a tgSpringCableActuator. Returns a list of size 1.
 */
std::vector<tgSensor*> tgSpringCableActuatorSensorInfo::createSensorsIfAppropriate(tgSenseable* pSenseable)
{
  //CHECK: the caller SHOULD HAVE made sure that pSenseable
  // was a tgSpringCableActuator pointer. If not, complain!!
  if (!isThisMySenseable(pSenseable)) {
    throw std::invalid_argument("pSenseable is NOT a tgSpringCableActuator, inside tgSpringCableActuatorSensorInfo.");
  }
  // Create the list we'll return
  std::vector<tgSensor*> newSensors;
  // Then, if the program hasn't quit, make the sensor.
  // Note that we cast the pointer here, knowing that it will succeed.
  newSensors.push_back( new tgSpringCableActuatorSensor( tgCast::cast<tgSenseable, tgSpringCableActuator>(pSenseable) ));
  return newSensors;
}
