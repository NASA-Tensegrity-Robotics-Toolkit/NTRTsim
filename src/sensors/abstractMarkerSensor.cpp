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
 * @file abstractMarkerSensor.cpp
 * @brief Implementation of the abstractMarkerSensor class.
 * @author Drew Sabelhaus
 * @date January 4, 2017
 * $Id$
 */

// This class:
#include "abstractMarkerSensor.h"

// Includes from NTRT:
#include "core/tgSenseable.h"
#include "core/tgCast.h"
#include "core/tgTags.h"

// Includes from the c++ standard library:
//#include <iostream>
#include <sstream>  
#include <stdexcept>
#include <cassert>
#include <string> // for std::to_string(float)

// Includes from Bullet Physics:
#include "LinearMath/btVector3.h"

/**
 * This class is a sensor for abstractMarkers.
 * Its constructor just calls tgSensor's constructor.
 */
abstractMarkerSensor::abstractMarkerSensor(abstractMarker* pAbstractMarker) : tgSensor(pAbstractMarker)
{
  // Note that this pointer may be 0 (equivalent to NULL) if the cast in
  // the calling function from tgSenseable to abstractMarker fails.
  if (pAbstractMarker == NULL) {
    throw std::invalid_argument("Pointer to pAbstractMarker is NULL inside abstractMarkerSensor.");
  }
}

/** 
 * A class with virtual member functions must have a virtual destructor. 
 * Note that management of the pointer to the abstractMarker (tgSenseable) is managed
 * by the destructor for tgSensor.
 */
abstractMarkerSensor::~abstractMarkerSensor()
{
}

/**
 * The two methods from tgSensor.
 * A abstractMarker can return its position and color.
 * It does not have any tags.
 * A suggested use here is to compare your markers by color
 * instead of tags - make markerA to be (1, 1, 0), then examine
 * the data for the (1, 1, 0) colored marker.
 * TO-DO: what's "node number"?
 */
std::vector<std::string> abstractMarkerSensor::getSensorDataHeadings() {
  // Note that this class has access to the parent's pointer, m_pSens.
  // Let's cast that to a pointer to a abstractMarker right now.
  // Here, "m_pAbstractMarker" stands for "my pointer to a abstractMarker."
  abstractMarker* m_pAbstractMarker = tgCast::cast<tgSenseable, abstractMarker>(m_pSens);
  // Check: if the cast failed, this will return 0.
  // In that case, this abstractMarkerSensor does not point to a abstractMarker!!!
  assert( m_pAbstractMarker != 0);

  // The list to which we'll append all the sensor headings:
  std::vector<std::string> headings;
  
  // Pull out the color for the marker,
  // that's what we'll include instead of a tag.
  btVector3 colorVec = m_pAbstractMarker->getColor();
  // Turn the color into a string for use later. Make a stringstream
  // that will then be converted into a string.
  std::stringstream colorStream;
  colorStream << "(" << colorVec.getX() << ", " << colorVec.getY() << ", "
	      << colorVec.getZ() << ")";
  std::string color = colorStream.str();
  

  //DEBUGGING
  std::cout << "creating an abstract marker sensor for a marker with color: "
	    << color << std::endl;
  // Copied from tgSensor.h:
  /**
   * Headings should have the following form:
   * The type of sensor, then an open parenthesis "(" and the tags
   * of the specific tgSenseable object, then a ")." and a label for the 
   * specific field that will be output in that row.
   * For example, if sensor will be sensing a abstractMarker 
   * with color (1, 1, 0), its label for the X position might be "abstractMarker((1,1,0)).X"
   */

  // The string 'prefix' will be added to each heading.
  std::string prefix = "abstractMarker(";

  // Note that the orientation is a btVector3 object of Euler angles,
  // which I believe are overloaded as strings...
  // Also, the XYZ positions are of the center of mass.
  // TO-DO: check which euler angles are which!!!
  
  headings.push_back( prefix + color + ").X" );
  headings.push_back( prefix + color + ").Y" );
  headings.push_back( prefix + color + ").Z" );

  // Return the resulting vector.
  return headings;
}

/**
 * The method that collects the actual data from this abstractMarker.
 */
std::vector<std::string> abstractMarkerSensor::getSensorData() {
  // Similar to getSensorDataHeading, cast the a pointer to a abstractMarker right now.
  abstractMarker* m_pAbstractMarker = tgCast::cast<tgSenseable, abstractMarker>(m_pSens);
  // Check: if the cast failed, this will return 0.
  // In that case, this abstractMarkerSensor does not point to a abstractMarker!!!
  assert( m_pAbstractMarker != 0);
  // Pick out the XYZ position of this abstractMarker, in the world frame.
  btVector3 pos = m_pAbstractMarker->getWorldPosition();

  // The list of sensor data that will be returned:
  std::vector<std::string> sensordata;

  /**
   * The original version of this section of code, which 
   * output one string, looked like:
   * std::stringstream sensordata;
   * sensordata << com[0] << ","
   *	     << com[1] << ","
   *	     << com[2] << ","
   *	     << orient[0] << ","
   *	     << orient[1] << ","
   *	     << orient[2] << ","
   *	     << m_pAbstractMarker->mass() << ",";
   */
  
  // The floats (btScalars?) need to be converted to strings
  // via a stringstream.
  std::stringstream ss;

  // pos[0]
  ss << pos[0];
  sensordata.push_back( ss.str() );
  // Reset the stream.
  ss.str("");
  
  // pos[1]
  ss << pos[1];
  sensordata.push_back( ss.str() );
  ss.str("");
  // pos[2]
  ss << pos[2];
  sensordata.push_back( ss.str() );
  ss.str("");
  
  return sensordata;
}

//end.
