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
 * @file abstractMarkerSensorInfo.cpp
 * @brief Contains the implementation of concrete class abstractMarkerSensorInfo
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "abstractMarkerSensorInfo.h"
// Other includes from NTRTsim
#include "abstractMarkerSensor.h"
#include "core/abstractMarker.h"
#include "core/tgSenseable.h"
#include "core/tgCast.h"
// Other includes from the C++ standard library
#include <stdexcept>

/**
 * Nothing to do in this constructor. A sensor info doesn't have any data.
 */
abstractMarkerSensorInfo::abstractMarkerSensorInfo()
{
}

/**
 * Similarly, empty destructor.
 */
abstractMarkerSensorInfo::~abstractMarkerSensorInfo()
{
}

/**
 * To check if a tgSenseable is a abstractMarker, we can use tgCast.
 */
bool abstractMarkerSensorInfo::isThisMySenseable(tgSenseable* pSenseable)
{
  // The following cast will return 0 if the senseable is not a abstractMarker.
  abstractMarker* m_pAbstractMarker = tgCast::cast<tgSenseable, abstractMarker>(pSenseable);
  if( m_pAbstractMarker == 0 )
    return 0;
  else {
    return 1;
  }
}

/**
 * Create a abstractMarker sensor for a abstractMarker. Returns a list of size 1.
 */
std::vector<tgSensor*> abstractMarkerSensorInfo::createSensorsIfAppropriate(tgSenseable* pSenseable)
{
  //CHECK: the caller SHOULD HAVE made sure that pSenseable
  // was a abstractMarker pointer. If not, complain!!
  if (!isThisMySenseable(pSenseable)) {
    throw std::invalid_argument("pSenseable is NOT a abstractMarker, inside abstractMarkerSensorInfo.");
  }
  // Create the list we'll return
  std::vector<tgSensor*> newSensors;
  // Then, if the program hasn't quit, make the sensor.
  // Note that we cast the pointer here, knowing that it will succeed.
  newSensors.push_back( new abstractMarkerSensor( tgCast::cast<tgSenseable, abstractMarker>(pSenseable) ));
  return newSensors;
}
