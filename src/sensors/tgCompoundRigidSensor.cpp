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
#include <math.h> // for the constant PI (3.14...)

// Includes from Bullet Physics:
#include "btBulletDynamicsCommon.h" // includes all the Bullet stuff we need.

//#include "LinearMath/btVector3.h"
//#include "LinearMath/btQuaternion.h" //already included in header.
//#include "BulletDynamics/Dynamics/btRigidBody.h" // accessing the btRigids inside m_rigids.

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
  // Confirm that there is at least one rigid.
  if( m_rigids.empty() ) {
    throw std::runtime_error("tgCompoundRigidSensor found no rigid bodies with its tag - something is wrong (inside constructor.)");
  }
  // TO-DO: better validation.
  
  // Next, get the initial orientation of the rigid body.
  // This will be needed for comparison later.
  // TO-DO: make sure this is called AFTER the rigid body is "moved into place"
  // ...is that what happens in NTRT/Bullet??

  // (1) get the first Bullet rigid body from the list
  btRigidBody* firstRigid = m_rigids[0]->getPRigidBody();
  // (2) store its current orientation
  origOrientQuat = firstRigid->getOrientation();
  // (3) have Bullet calculate the inverse quaternion.
  //     That's what will be used below.
  origOrientQuatInv = origOrientQuat.inverse();
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
// NEED TO DO ASAP: this is INCORRECT, we need to average / integrate
//   over all the volume of the rigids, not just avg their individual COMs.
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
  // For now, it will be easier to poke around at the underlying Bullet Physics
  // objects.
  // TO-DO: encapsulate this functionality inside tgBaseRigid.
  // To get the "difference" between wherever the first rigid body was,
  // and where it is now, multiply Q_curr * inv(Q_0).
  // First, get the orientation of the underlying Bullet rigid body:
  btQuaternion currentOrientQuat = m_rigids[0]->getPRigidBody()->getOrientation();
  // The "difference" is then
  btQuaternion diffOrientQuat = currentOrientQuat * origOrientQuatInv;
  // Convert to roll/pitch/yaw just like inside tgBaseRigid::orientation().
  btMatrix3x3 rotMat = btMatrix3x3( diffOrientQuat );
  btScalar yaw = 0.0;
  btScalar pitch = 0.0;
  btScalar roll = 0.0;
  rotMat.getEulerYPR(yaw, pitch, roll);
  // Convert from radians to degrees, since that's what most people
  // will care about when parsing this data.
  yaw = 180/M_PI * yaw;
  pitch = 180/M_PI * pitch;
  roll = 180/M_PI * roll;  

  // The final result:
  btVector3 orient = btVector3(yaw, pitch, roll);
  return orient;
  
  // The resulting vector:
  //btVector3* orient = new btVector3(0.0, 0.0, 0.0);

  //return *orient;
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
  headings.push_back( prefix + m_tag + ").YawDegrees" );
  headings.push_back( prefix + m_tag + ").PitchDegrees" );
  headings.push_back( prefix + m_tag + ").RollDegrees" );
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
  btVector3 orient = getOrientation();
  
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

  // yaw
  ss << orient[0];
  sensordata.push_back( ss.str() );
  ss.str("");

  // pitch
  ss << orient[1];
  sensordata.push_back( ss.str() );
  ss.str("");

  // roll
  ss << orient[2];
  sensordata.push_back( ss.str() );
  ss.str("");
  
  //sensordata.push_back( "" );
  //sensordata.push_back( "" );
  //sensordata.push_back( "" );

  // mass
  ss << getMass();
  sensordata.push_back( ss.str() );
  ss.str("");
  
  return sensordata;
}

//end.
