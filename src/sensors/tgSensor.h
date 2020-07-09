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
 * @file tgSensor.h
 * @brief Constains definition of abstract class tgSensor, which defines the methods that a sensor must implement.
 * @author Drew Sabelhaus
 * $Id$
 */

#ifndef TG_SENSOR_H
#define TG_SENSOR_H

// Forward-declare the tgSenseable class,
// so that we can have pointers to it.
class tgSenseable;

// From the C++ standard library:
#include <iostream> //for strings
#include <vector> // for returning lists of strings

/**
 * This class defines methods for use with sensors.
 * Any individual sensor (ex., a tgRodSensor) will need to re-implement
 * both of these methods.
 * Note that we make everything pure virtual here to force re-definition.
 * Sensing data from objects occurs in two places in the NTRTsim workflow.
 * First, when setting up the simulation, a heading for the data is given.
 * This describes the data that will be returned.
 * Second, when the simulation is running, the data itself can be taken.
 * Note that it's up to the caller (e.g., a tgDataLogger2) to match up the headings
 * with the data.
 */
class tgSensor
{
public:

  /**
   * This constructor takes a pointer to a sense-able object.
   * Note that this creates complications with casting: in child classes,
   * this pointer will need to be casted to the particular type of sense-able
   * object that a sensor works on.
   * The reason why we declare it here as a pointer to the parent class
   * is for setup reasons: we want a tgDataLogger2 to be able to ask this
   * class, "Can you sense a particular tgSenseable?".
   * The way to do that is to pass in a pointer of the parent type,
   * and attempt a cast to the child type via a call to tgCast.
   */
  tgSensor(tgSenseable* pSens);

  // Classes with virtual member functions must also have virtual destructors.
  virtual ~tgSensor();

  /**
   * Create a descriptive heading for all the data that this class can return.
   * This will be a vector of strings, with each string being a heading.
   * Headings should have the following form:
   * The type of sensor, then an open parenthesis "(" and the tags
   * of the specific tgSenseable object, then a ")." and a label for the 
   * specific field that will be output in that row.
   * For example, if sensor will be sensing a rod 
   * with tags "t4 t5", its label for the X position might be "rod(t4 t5).X"
   * @return a list of strings, each being a descriptive heading for a
   * specific piece of data that will be returned.
   */
  virtual std::vector<std::string> getSensorDataHeadings() = 0;

  /**
   * Return the data from this class itself.
   * Note that this MUST have the same number of elements as is returned by
   * the getDataHeading function.
   * @return a list of strings, each being a piece of sensor data,
   * in the same order as the headings.
   */
  virtual std::vector<std::string> getSensorData() = 0;

  // TO-DO: should any of this be const?

protected:

  /**
   * This class stores a pointer to its tgSenseable object.
   * Note that it is protected so that subclasses can access it,
   * but children will still need to cast it against their specific
   * type of tgSenseable (e.g., rod, cable, etc.)
   */
  tgSenseable* m_pSens;

};

#endif //TG_SENSOR_H
