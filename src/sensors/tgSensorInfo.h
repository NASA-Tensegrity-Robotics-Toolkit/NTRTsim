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

#ifndef TG_SENSOR_INFO_H
#define TG_SENSOR_INFO_H

/**
 * @file tgSensorInfo.h
 * @brief Definition of abstract class tgSensorInfo 
 * @author Drew Sabelhaus
 * @date January 2017
 * $Id$
 */

// Includes:
// The C++ Standard Library
#include <iostream> // for the to-string overloaded method
#include <vector> // for returning lists of sensors
// This library
// ...
// Bullet Physics
// ...

// Forward references
class tgSenseable;
class tgSensor;

/**
 * tgSensorInfo is an abstract class that contains methods for creating sensors.
 * This is how a tgDataManager will create sensors for specific tgSenseable objects
 * (e.g., how a tgDataLogger2 will create sensors for specific tgModels.)
 * Note that an individual tgSensorInfo may return multiple sensors upon a single
 * call to createSensorsIfAppropriate - this would be the case for e.g. tgCompoundRigidSensor,
 * but in most cases (e.g. rods and spring-cables) only one sensor is returned.
 */ 
class tgSensorInfo
{
public:
        
  /**
   * Empty constructor and destructor.
   * Note that the destructor must be virtual also, since this is a class with
   * virtual member functions.
   */
  tgSensorInfo();
  virtual ~tgSensorInfo();

  /**
   * First, a tgSensorInfo should be able to determine if
   * it can create at least one sensor for a specific tgSenseable object.
   * This is the first step in creating sensors, first check then create.
   * @param[in] pSenseable a pointer to a senseable object (usually a tgModel), which either can or cannot be sensed by whatever particular sensor would be created by this tgSensorInfo.
   * @return if it's possible for this sensor info to create a sensor for the given tgSenseable object.
   */
  virtual bool isThisMySenseable(tgSenseable* pSenseable) = 0;

  /**
   * Next, this class needs to be able to actually create one or more sensors.
   * @param[in] pSenseable a pointer to the tgSenseable object that is to be sensed.
   * @return a vector of pointers to any sensors that were created. This list can have zero elements,
   * for example in the case that all possible sensors have already been created for an object.
   * Note that it's up to the subclasses to make sure that redundant sensors are not created.
   * One way to do this, for example, is to only create sensors for the "leaf" descendants.
   * That would be how (for example) tgRodSensorInfo works: by only creating sensors for
   * tgSenseables that can be cast to tgRods, we can be sure that only one sensors is made per rod,
   * since any individual rod will only be passed in ONCE to this function.
   * However, for more complicated situations, like tgCompoundRigidSensor, a subclass would need
   * to keep track of e.g. which compound rigids have been assigned a sensor, so as to not
   * create multiple for the same compound rigid.
   * In short, return the least number of sensors possible, and if dealing directly with children
   * of tgModel, then don't create any sensors for the tgModel itself, just its children.
   */
  virtual std::vector<tgSensor*> createSensorsIfAppropriate(tgSenseable* pSenseable) = 0;

};


/**
 * Overload operator<<() to handle tgSensorInfo
 * @param[in,out] os an ostream
 * @param[in] a reference to a tgSensorInfo
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const tgSensorInfo& obj)
{
    os << "tgSensorInfo";
    return os;
}

#endif // TG_SENSOR_INFO_H
