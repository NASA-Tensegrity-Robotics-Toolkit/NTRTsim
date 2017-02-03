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
 * @file tgCompoundRigidSensor.cpp
 * @brief Implementation of the tgCompoundRigidSensor class.
 * @author Drew Sabelhaus
 * @date January 4, 2017
 * $Id$
 */

// This class:
#include "tgCompoundRigidSensor.h"

// Includes from NTRT:
#include "core/tgSenseable.h"
#include "core/tgCast.h"
#include "core/tgTags.h"
#include "core/tgBaseRigid.h"

// Includes from the c++ standard library:
//#include <iostream>
#include <sstream>  
#include <stdexcept>
#include <cassert>
#include <string> // for std::to_string(float)

// Includes from Bullet Physics:
#include "LinearMath/btVector3.h"

/**
 * This class is a sensor for tgCompoundRigids.
 * The constructor with only a pointer will always fail,
 * since it's required that a compound tag be passed in too,
 * otherwise either (a) the sensor doesn't know which compound to sense,
 * or (b) if the sensor will sense ALL the compounds, that could create
 * duplicate sensors for the same compound and also breaks the 1-sensor-per
 * -object rule.
 */
tgCompoundRigidSensor::tgCompoundRigidSensor(tgModel* pModel) : tgSensor(pModel)
{
  throw std::invalid_argument("Called the constructor for tgCompoundRigidSensor with only a pointer, NOT a tag. That's not allowed, a specific compound tag must be passed in also.");
}

/**
 * The actual constructor. This calls the parent, then populates the
 * list of 
 */
tgCompoundRigidSensor::tgCompoundRigidSensor(tgModel* pModel, std::string tag) :
  tgSensor(pModel),
  m_tag(tag)
{
  // Note that this pointer may be 0 (equivalent to NULL) if the cast in
  // the calling function from tgSenseable to tgModel fails.
  if (pModel == NULL) {
    throw std::invalid_argument("Pointer to pModel is NULL inside tgCompoundRigidSensor.");
  }
  // Then, pick out the rigid bodies in the model that have the tag.
  // tgModel has a nice method for doing this.
  m_rigids = pModel->find<tgBaseRigid>(m_tag);
  // TO-DO: some kind of validation?? Do m_rigids have more than one item, for example?
}

/** 
 * A class with virtual member functions must have a virtual destructor. 
 * Note that management of the pointer to the tgModel (tgSenseable) is managed
 * by the destructor for tgSensor.
 */
tgCompoundRigidSensor::~tgCompoundRigidSensor()
{
  // However, we should clear the pointers to the rigid bodies.
  // Again, not delete the objects, just clear the pointers.
  // Note that tgDataManager calls delete on this sensor during reset
  // e.g., (teardown/setup.)

  // TO-DO: implement this.
}

// Three data collection methods. They should all
// return zeroes if m_rigids is empty.
btVector3 tgCompoundRigidSensor::getCenterOfMass()
{
  // This method takes the average of all the centers of mass.
  // It should be sufficient to just add then divide each component
  // of the 3D vector.
  // The resulting vector:
  btVector3* pCom = new btVector3(0.0, 0.0, 0.0);
  // Iterate and add all the centers of mass of the components.
  for( size_t i=0; i < m_rigids.size(); i++){
    *pCom += m_rigids[i]->centerOfMass();
  }
  // Average the components:
  *pCom /= m_rigids.size();

  return *pCom;
}

btVector3 tgCompoundRigidSensor::getOrientation()
{
  // The resulting vector:
  btVector3* orient = new btVector3(0.0, 0.0, 0.0);

  return *orient;
}

double tgCompoundRigidSensor::getMass()
{
  // Add the mass of all the rigid bodies.
  double mass = 0;
  for( size_t i=0; i < m_rigids.size(); i++){
    mass += m_rigids[i]->mass();
  }
  return mass;
}

/**
 * The two methods from tgSensor.
 * A compound rigid body has a position (center of mass of all bodies) as
 * wel as an orientation (which we'll define in a certain way below).
 * Also, the previous tgDataObserver recorded the mass of the rigid bodies too,
 * so do that here for consistency. (even though mass doesn't change with time.)
 */
std::vector<std::string> tgCompoundRigidSensor::getSensorDataHeadings() {
  // Note that this class has access to the parent's pointer, m_pSens.
  // Let's cast that to a pointer to a tgModel right now.
  // Here, "m_pModel" stands for "my pointer to a tgModel."
  tgModel* m_pModel = tgCast::cast<tgSenseable, tgModel>(m_pSens);
  // Check: if the cast failed, this will return 0.
  // In that case, this tgCompoundRigidSensor does not point to a tgModel!!!
  assert( m_pModel != 0);

  // The list to which we'll append all the sensor headings:
  std::vector<std::string> headings;

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
  std::string prefix = "compound(";

  // Create the headings, similar to tgRod for example.
  headings.push_back( prefix + m_tag + ").X" );
  headings.push_back( prefix + m_tag + ").Y" );
  headings.push_back( prefix + m_tag + ").Z" );
  headings.push_back( prefix + m_tag + ").Euler1" );
  headings.push_back( prefix + m_tag + ").Euler2" );
  headings.push_back( prefix + m_tag + ").Euler3" );
  headings.push_back( prefix + m_tag + ").mass" );

  // Return the resulting vector.
  return headings;
}

/**
 * The method that collects the actual data from this compound rigid body.
 */
std::vector<std::string> tgCompoundRigidSensor::getSensorData() {
  // Note that this method uses m_rigids directly, no need to deal
  // with the parent class' pointer to m_pSens.
  
  // The floats (btScalars?) need to be converted to strings
  // via a stringstream.
  std::stringstream ss;

  // The list of sensor data that will be returned:
  std::vector<std::string> sensordata;

  // Get the position and orientation of this compound body.
  // Call the helper functions
  btVector3 com = getCenterOfMass();

  // com[0]
  ss << com[0];
  sensordata.push_back( ss.str() );
  // Reset the stream.
  ss.str("");
  
  // com[1]
  ss << com[1];
  sensordata.push_back( ss.str() );
  ss.str("");
  // com[2]
  ss << com[2];
  sensordata.push_back( ss.str() );
  ss.str("");

  // Add three empty strings until we figure out what to do
  // about orientation.
  sensordata.push_back( "" );
  sensordata.push_back( "" );
  sensordata.push_back( "" );

  // mass
  ss << getMass();
  sensordata.push_back( ss.str() );
  ss.str("");
  
  return sensordata;
}

//end.
