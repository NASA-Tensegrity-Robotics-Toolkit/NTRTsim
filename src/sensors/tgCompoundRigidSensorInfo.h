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

#ifndef TG_COMPOUND_RIGID_SENSOR_INFO_H
#define TG_COMPOUND_RIGID_SENSOR_INFO_H

/**
 * @file tgCompoundRigidSensorInfo.h
 * @brief Definition of concrete class tgCompoundRigidSensorInfo 
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
//class tgCompoundRigidSensor;
class tgSensor;

/**
 * tgCompoundRigidSensorInfo is a sensor info class that creates sensors for
 * compound rigid bodies. It creates a sensor for any tgSenseable object that:
 * (1) is a tgModel with at least two descendants
 * (2) at least two of the descendants, both of which can be cast to tgBaseRigids,
 *     have matching "compound_XXXXXX" tags
 */
class tgCompoundRigidSensorInfo : public tgSensorInfo
{
 public:

  /**
   * tgSensorInfo has an empty constructor. It does not contain any data.
   */
  tgCompoundRigidSensorInfo();

  /** 
   * Similarly, its destructor should be empty.
   */
  ~tgCompoundRigidSensorInfo();

  /**
   * From tgSensorInfo, need to implement a check to see if a particular
   * tgSenseable fits the criteria for making a compound rigid body sensor.
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
   */
  virtual tgSensor* createSensor(tgSenseable* pSenseable);

};

#endif // TG_COMPOUND_RIGID_SENSOR_INFO_H
