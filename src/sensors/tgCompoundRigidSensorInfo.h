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

// Forward references
class tgSenseable;
//class tgCompoundRigidSensor; // since createSensor returns only a tgSensor.
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
   *
   * Problem: how to make it so that EXACTLY ONE tgCompoundRigidSensor is made
   * per single compound rigid body?
   * This is complicated by the fact that tgModels can have other tgModels.
   * Take the spine in YAML example. The root tgModel has four other tgModels in it,
   * each of which contain four rods. Each of these four rods are compounds.
   *      (note that no compounds exist cross-tgModel or up/down the heirarchy.)
   *
   * Solution: state machine inside tgCompoundRigidSensor.
   * (1) When isThisMySenseable, if no root tgModel exists, declare the passed-in
   * as root.
   * (2) If the passed-in tgModel 
   *      (a) has a compound in it, and 
   *      (b) doesn't have any child container tgModels - e.g., only has rods and rigids and whatnot, 
   * ...then:
   *      (c) record the hash of the compounds that will be created
   *      (d) return true
   *
   * Then, when createSensor is called:
   * (1) again, check somehow that this is the leaf-iest tgModel
   * (2) create whatever sensors can be created under this leaf model
   *     ...note that this will return a sensor to POTENTIALLY MULTIPLE 
   *        compound rigid bodies, if for example multiple compounds exist
   *        within the same level of a tgModel (see, for ex., VertSpineModel.)
   *
   * Need to handle the case where there is a tgModel with one or more compounds
   * in it, but it also has a child tgModel with one or more compounds in it.
   * So, for example, we can't just say "Iff a leaf tgModel, then make the sensor",
   * since there could be a case when the model has a sensor for its compound
   * but its children should supply the sensor(s) for their compounds.
   *      THIS REQUIRES THE PASSING-IN OF COMPOUND TAGS TO THE SENSOR CONSTRUCTOR.
   *      Otherwise, in this case, how would the sensor know what parts to sense?
   *      If it was to sense "everything", then it would sense the parent's compound
   *      as well as the child's, but there will also be another sensor made for the
   *      child tgModel that senses its compound(s).
   *
   * Need to find a way to keep consistency between calls to isThisMySenseable and
   * createSensor.
   *
   * Also, is this too complicated????
   * Should we just keep a list of what compounds have been sensed, and be greedy
   * with allocating sensors to tgModels?
   * For example, we say "as soon as we see a certain compound, make a sensor for it,
   * then add it to a blacklist."
   * That might be easiest, although it's probably not what the message-passing
   * people would want.
   */
  virtual bool isThisMySenseable(tgSenseable* pSenseable);

  /**
   * Similarly, create a sensor if appropriate.
   * See tgSensorInfo for more... info.
   * @param[in] pSenseable pointer to a senseable object. Sensor will be created
   * for this pSenseable.
   */
  virtual tgSensor* createSensor(tgSenseable* pSenseable);

 private:

  /**
   * Keep track of which compounds have already been accounted for
   * during sensor creation.
   * This blacklist will be checked before passing in compound tags to the
   * constructor for tgCompoundRigidSensor.
   * That way, we avoid the case where multiple nested tgModels have 
   * separate tgCompoundRigidSensors for the same compound.
   */
  std::vector<std::string> blacklist;

};

#endif // TG_COMPOUND_RIGID_SENSOR_INFO_H
