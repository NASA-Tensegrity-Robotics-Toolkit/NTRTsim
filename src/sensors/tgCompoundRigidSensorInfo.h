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
#include <vector>
#include <string>
#include <map> // for the helper function
// Includes from Boost
//#include <boost/regex.hpp>

// Forward references
class tgSenseable;
//class tgCompoundRigidSensor; // since createSensor returns only a tgSensor.
class tgSensor;
class tgModel; // this class only works on tgModels, since tgSenseables don't have tags.

/**
 * tgCompoundRigidSensorInfo is a sensor info class that creates sensors for
 * compound rigid bodies. It creates a sensor for any tgSenseable object that:
 * (1) is a tgModel with at least two descendants
 * (2) at least two of the descendants, both of which can be cast to tgBaseRigids,
 *     have matching "compound_XXXXXX" tags
 * See the implementation of these functions for more details about how many sensors
 * are created. If used properly (e.g. this is the only tgCompoundRigidSensorInfo in the
 * application), there should not be any redundant sensors created.
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
   * This returns true iff the passed-in model has at least one compound, and that
   * compound is not in the blacklist.
   * @param[in] pSenseable a pointer to a tgSenseable object, that this sensor info
   * may or may not be able to create a sensor for.
   * @return true if pSenseable is able to be sensed by the type of sensor that
   * this sensor info creates.
   */
  virtual bool isThisMySenseable(tgSenseable* pSenseable);

  /**
   * Similarly, create a sensor if appropriate.
   * This class can make zero or more sensors via this method.
   * An empty list will be returned if there are no available compounds in 
   * the passed-in model that are not in the blacklist.
   * Otherwise, it will create an individual sensor for every compound that is
   * not in the blacklist.
   * @param[in] pSenseable pointer to a senseable object. Sensor will be created
   * for this pSenseable.
   * @return zero or more new sensors.
   */
  virtual std::vector<tgSensor*> createSensorsIfAppropriate(tgSenseable* pSenseable);

 private:

  /**
   * A helper function that extracts any compound tags from a tgModel object.
   * This is used in multiple places in this class.
   * @param[in] pModel a pointer to a tgModel, which may or may not have any compound
   *    objects or compound tags.
   * @return a map of the compound tags in the model, and the count of how many
   *    descendants have those tags. Counting the number of tags doesn't take much
   *    overhead, and is very useful for some purposes (e.g. in checking isThisMySenseable.)
   */
  std::map<std::string, int> getCompoundTags(tgModel* pModel);

  /**
   * Keep track of which compounds have already been accounted for
   * during sensor creation.
   * This blacklist will be checked before passing in compound tags to the
   * constructor for tgCompoundRigidSensor.
   * That way, we avoid the case where multiple nested tgModels have 
   * separate tgCompoundRigidSensors for the same compound.
   */
  std::vector<std::string> blacklist;

  /**
   * a helper that encapsulates the call to std::find in the blacklist.
   */
  bool isBlacklisted(std::string tag);
  
};

#endif // TG_COMPOUND_RIGID_SENSOR_INFO_H
