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
 * @file tgRodSensor.cpp
 * @brief Implementation of the tgRodSensor class.
 * @author Drew Sabelhaus
 * @date January 4, 2017
 * $Id$
 */

// This class:
#include "tgRodSensor.h"

// Includes from NTRT:
#include "core/tgSenseable.h"
#include "core/tgCast.h"
#include "core/tgTags.h"

// Includes from the c++ standard library:
//#include <iostream>
#include <sstream>  
#include <stdexcept>
#include <cassert>

// Includes from Bullet Physics:
#include "LinearMath/btVector3.h"

/**
 * This class is a sensor for tgRods.
 * Its constructor just calls tgSensor's constructor.
 */
tgRodSensor::tgRodSensor(tgRod* pRod) : tgSensor(pRod)
{
  // Note that this pointer may be 0 (equivalent to NULL) if the cast in
  // the calling function from tgSenseable to tgRod fails.
  if (pRod == NULL) {
    throw std::invalid_argument("Pointer to pRod is NULL inside tgRodSensor.");
  }
}

/** 
 * A class with virtual member functions must have a virtual destructor. 
 * Note that management of the pointer to the tgRod (tgSenseable) is managed
 * by the destructor for tgSensor.
 */
tgRodSensor::~tgRodSensor()
{
}

/**
 * The two methods from tgSensor.
 * A tgRod can return its position and orientation.
 * Also, the previous tgDataObserver recorded the mass of the rod too,
 * so do that here for consistency. (even though mass doesn't change with time.)
 */
std::string tgRodSensor::getSensorDataHeading(std::string prefix) {
  // Note that this class has access to the parent's pointer, m_pSens.
  // Let's cast that to a pointer to a tgRod right now.
  // Here, "m_pRod" stands for "my pointer to a tgRod."
  tgRod* m_pRod = tgCast::cast<tgSenseable, tgRod>(m_pSens);
  // Check: if the cast failed, this will return 0.
  // In that case, this tgRodSensor does not point to a tgRod!!!
  assert( m_pRod != 0);
  
  // Create a string stream to which we'll append all the
  // information.
  std::stringstream heading;
  // Pull out the tags for this rod, so we only have to call the accessor once.
  //std::deque<std::string> m_tags = m_pRod->getTags();
  tgTags m_tags = m_pRod->getTags();

  // Copied from tgSensor.h:
  /**
   * A sensor heading should have:
   * (a) a series of comma-separated values in a row, (b) each "column" of
   * the CSV is prepended with "prefix" and then a "_", (c) then has
   * the type of sensor, then an open parenthesis "(" and the tags
   * of the specific tgSenseable object, then a ")." and a label for the 
   * specific field that will be output in that row.
   * For example, if sensor 4 (the prefix) will be sensing a rod 
   * with tags "t4 t5", its label for the X position might be "4_rod(t4 t5).X"
   */

  // The string 'prefix' will be added to each column. Usually, this would
  // be for something like a numbering system that a data manager would
  // implement.
  // Needs an underscore to separate it from the rest of the heading.
  prefix = prefix + "_rod(";

  // Note that the orientation is a btVector3 object of Euler angles,
  // which I believe are overloaded as strings...
  // Also, the XYZ positions are of the center of mass.
  // TO-DO: check which euler angles are which!!!
  heading << prefix << m_tags << ").X,"
	  << prefix << m_tags << ").Y,"
	  << prefix << m_tags << ").Z,"
	  << prefix << m_tags << ").Euler1,"
	  << prefix << m_tags << ").Euler2,"
	  << prefix << m_tags << ").Euler3,"
	  << prefix << m_tags << ").mass,";

  // Return the string version of this string stream.
  return heading.str();
}

/**
 * The method that collects the actual data from this tgRod.
 */
std::string tgRodSensor::getSensorData() {
  // Similar to getSensorDataHeading, cast the a pointer to a tgRod right now.
  tgRod* m_pRod = tgCast::cast<tgSenseable, tgRod>(m_pSens);
  // Check: if the cast failed, this will return 0.
  // In that case, this tgRodSensor does not point to a tgRod!!!
  assert( m_pRod != 0);
  // Pick out the XYZ position of the center of mass of this rod.
  btVector3 com = m_pRod->centerOfMass();
  btVector3 orient = m_pRod->orientation();
  // Note that the 'orientation' method also returns a btVector3.
  
  // Similar to the heading, create a string steam
  // of all the data to be returned.
  std::stringstream sensordata;
  sensordata << com[0] << ","
	     << com[1] << ","
	     << com[2] << ","
	     << orient[0] << ","
	     << orient[1] << ","
	     << orient[2] << ","
	     << m_pRod->mass() << ",";
  // Again, must return a string, not a stringstream.
  return sensordata.str();
}

//end.
