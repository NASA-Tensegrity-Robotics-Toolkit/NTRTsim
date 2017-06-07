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

#ifndef TG_SPRING_CABLE_ACTUATOR_SENSOR_INFO_H
#define TG_SPRING_CABLE_ACTUATOR_SENSOR_INFO_H

/**
 * @file tgSpringCableActuatorSensorInfo.h
 * @brief Definition of concrete class tgSpringCableActuatorSensorInfo 
 * @author Drew Sabelhaus
 * @date January 2017
 * $Id$
 */

// This module
#include "tgSensorInfo.h"
// Other includes from NTRTsim
// ...
// Other includes from the C++ standard library
// ...

// Forward references
class tgSenseable;
class tgSpringCableActuator;
class tgSpringCableActuatorSensor;
class tgSensor;

/**
 * tgSpringCableActuatorSensorInfo is a sensor info class that creates 
 * tgSpringCableActuatorSensors for tgSpringCableActuators.
 */
class tgSpringCableActuatorSensorInfo : public tgSensorInfo
{
 public:

  /**
   * tgSensorInfo has an empty constructor. It does not contain any data.
   */
  tgSpringCableActuatorSensorInfo();

  /** 
   * Similarly, its destructor should be empty.
   */
  ~tgSpringCableActuatorSensorInfo();

  /**
   * From tgSensorInfo, need to implement a check to see if a particular
   * tgSenseable is a tgSpringCableActuator.
   * @param[in] pSenseable a pointer to a tgSenseable object, that this sensor info
   * may or may not be able to create a sensor for.
   * @return true if pSenseable is able to be sensed by the type of sensor that
   * this sensor info creates.
   */
  virtual bool isThisMySenseable(tgSenseable* pSenseable);

  /**
   * Similarly, create a sensor if appropriate.
   * See tgSensorInfo for more... info.
   * @param[in] pSenseable pointer to a senseable object. Sensor will be created
   * for this pSenseable.
   * @return a list of pointers to tgSpringCableActuatorSensors. 
   * Note that this should ALWAYS have size 1. Don't create sensors for 
   * elements that just have rods as descendants, only create sensors for 
   * an actual spring-cable actuator. (This eliminates redundant sensor creation.)
   * @throws invalid_argument if pSenseable is not a tgSpringCableActuator. 
   * This enforces the caller to check isThisMySenseable before creating sensors.
   */
  virtual std::vector<tgSensor*> createSensorsIfAppropriate(tgSenseable* pSenseable);

};

#endif // TG_SPRING_CABLE_ACTUATOR_SENSOR_INFO_H
