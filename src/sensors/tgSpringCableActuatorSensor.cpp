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
 * @file tgSpringCableActuatorSensor.cpp
 * @brief Implementation of the tgSpringCableActuatorSensor class.
 * @author Drew Sabelhaus
 * @date January 8, 2017
 * $Id$
 */

// This class:
#include "tgSpringCableActuatorSensor.h"

// Includes from NTRT:
#include "core/tgSenseable.h"
#include "core/tgCast.h"
#include "core/tgTags.h"

// Includes from the c++ standard library:
#include <sstream>  
#include <stdexcept>
#include <cassert>
#include <string> // for std::to_string(float)

// Includes from Bullet Physics:
#include "LinearMath/btVector3.h"

/**
 * This class is a sensor for tgSpringCableActuators.
 * Its constructor just calls tgSensor's constructor.
 */
tgSpringCableActuatorSensor::tgSpringCableActuatorSensor(tgSpringCableActuator* pSCA) : tgSensor(pSCA)
{
  // Note that this pointer may be 0 (equivalent to NULL) if the cast in
  // the calling function from tgSenseable to tgSpringCableActuator fails.
  if (pSCA == NULL) {
    throw std::invalid_argument("Pointer to pSCA is NULL inside tgSpringCableActuatorSensor.");
  }
}

/** 
 * A class with virtual member functions must have a virtual destructor. 
 * Note that management of the pointer to the 
 * tgSpringCableActuator (tgSenseable) is managed by the destructor for tgSensor.
 */
tgSpringCableActuatorSensor::~tgSpringCableActuatorSensor()
{
}

/**
 * The two methods from tgSensor.
 * A tgSpringCableActuator can return its rest length, current length, and tension.
 * TO-DO: somehow include the config for a specific tgSCA, like spring constant
 * and damping constant.
 */
std::vector<std::string> tgSpringCableActuatorSensor::getSensorDataHeadings() {
  // Note that this class has access to the parent's pointer, m_pSens.
  // Let's cast that to a pointer to a tgSpringCableActuator right now.
  // Here, "m_pSCA" stands for "my pointer to a tgSpringCableActuator."
  tgSpringCableActuator* m_pSCA =
    tgCast::cast<tgSenseable, tgSpringCableActuator>(m_pSens);
  // Check: if the cast failed, this will return 0.
  // In that case, this tgSpringCableActuatorSensor does
  // not point to a tgSpringCableActuator!!!
  assert( m_pSCA != 0);

  // The list to which we'll append all the sensor headings:
  std::vector<std::string> headings;

  // Pull out the tags for this spring cable actuator,
  // so we only have to call the accessor once.
  tgTags m_tags = m_pSCA->getTags();

  // Copied from tgSensor.h:
  /**
   * Headings should have the following form:
   * The type of sensor, then an open parenthesis "(" and the tags
   * of the specific tgSenseable object, then a ")." and a label for the 
   * specific field that will be output in that row.
   * For example, if sensor will be sensing a rod 
   * with tags "t4 t5", its label for the X position might be "rod(t4 t5).X"
   */

  // The string 'prefix' will be added to each heading.
  std::string prefix = "SCA(";

  // This SCA will give a rest length, current length, and tension.
  headings.push_back( prefix + m_tags + ").RestLen" );
  headings.push_back( prefix + m_tags + ").CurrLen" );
  headings.push_back( prefix + m_tags + ").Tension" );

  // Return the string version of this string stream.
  return headings;
}

/**
 * The method that collects the actual data from this tgSpringCableActuator.
 */
std::vector<std::string> tgSpringCableActuatorSensor::getSensorData() {
  // Similar to getSensorDataHeading, cast the a pointer
  // to a tgSpringCableActuator right now.
  tgSpringCableActuator* m_pSCA =
    tgCast::cast<tgSenseable, tgSpringCableActuator>(m_pSens);
  // Check: if the cast failed, this will return 0.
  // In that case, this tgSpringCableActuatorSensor does not point
  // to a tgSpringCableActuator!!!
  assert( m_pSCA != 0);

  // The list of sensor data that will be returned:
  std::vector<std::string> sensordata;

  // The floats doubles need to be converted to strings via a stringstream.
  std::stringstream ss;

  // rest length
  ss << m_pSCA->getRestLength();
  sensordata.push_back( ss.str() );
  // Reset the stream.
  ss.str("");

  // current length
  ss << m_pSCA->getCurrentLength();
  sensordata.push_back( ss.str() );
  ss.str("");

  // tension
  ss << m_pSCA->getTension();
  sensordata.push_back( ss.str() );
  ss.str("");
  
  return sensordata;
}

//end.
