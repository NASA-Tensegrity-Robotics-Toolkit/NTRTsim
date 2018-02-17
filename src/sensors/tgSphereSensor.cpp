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
 * @file tgSphereSensor.cpp
 * @brief Implementation of the tgSphereSensor class.
 * @author Drew Sabelhaus
 * @date January 4, 2017
 * $Id$
 */

// This class:
#include "tgSphereSensor.h"

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
#include "btBulletDynamicsCommon.h" //for collision shapes etc.

/**
 * This class is a sensor for tgSpheres.
 * Its constructor just calls tgSensor's constructor.
 */
tgSphereSensor::tgSphereSensor(tgSphere* pSphere) : tgSensor(pSphere)
{
  // Note that this pointer may be 0 (equivalent to NULL) if the cast in
  // the calling function from tgSenseable to tgSphere fails.
  if (pSphere == NULL) {
    throw std::invalid_argument("Pointer to pSphere is NULL inside tgSphereSensor.");
  }
}

/** 
 * A class with virtual member functions must have a virtual destructor. 
 * Note that management of the pointer to the tgSphere (tgSenseable) is managed
 * by the destructor for tgSensor.
 */
tgSphereSensor::~tgSphereSensor()
{
}

/**
 * The two methods from tgSensor.
 * A tgSphere can return its position and orientation.
 * Also, the previous tgDataObserver recorded the mass of the sphere too,
 * so do that here for consistency. (even though mass doesn't change with time.)
 */
std::vector<std::string> tgSphereSensor::getSensorDataHeadings() {
  // Note that this class has access to the parent's pointer, m_pSens.
  // Let's cast that to a pointer to a tgSphere right now.
  // Here, "m_pSphere" stands for "my pointer to a tgSphere."
  tgSphere* m_pSphere = tgCast::cast<tgSenseable, tgSphere>(m_pSens);
  // Check: if the cast failed, this will return 0.
  // In that case, this tgSphereSensor does not point to a tgSphere!!!
  assert( m_pSphere != 0);

  // The list to which we'll append all the sensor headings:
  std::vector<std::string> headings;
  
  // Pull out the tags for this sphere, so we only have to call the accessor once.
  tgTags m_tags = m_pSphere->getTags();

  // Copied from tgSensor.h:
  /**
   * Headings should have the following form:
   * The type of sensor, then an open parenthesis "(" and the tags
   * of the specific tgSenseable object, then a ")." and a label for the 
   * specific field that will be output in that row.
   * For example, if sensor will be sensing a sphere 
   * with tags "t4 t5", its label for the X position might be "sphere(t4 t5).X"
   */

  // The string 'prefix' will be added to each heading.
  std::string prefix = "sphere(";

  // Note that the orientation is a btVector3 object of Euler angles,
  // which I believe are overloaded as strings...
  // Also, the XYZ positions are of the center of mass.
  // TO-DO: check which euler angles are which!!!
  
  headings.push_back( prefix + m_tags + ").X" );
  headings.push_back( prefix + m_tags + ").Y" );
  headings.push_back( prefix + m_tags + ").Z" );
  headings.push_back( prefix + m_tags + ").mass" );

  // Return the resulting vector.
  return headings;
}

/**
 * The method that collects the actual data from this tgSphere.
 */
std::vector<std::string> tgSphereSensor::getSensorData() {
  // Similar to getSensorDataHeading, cast the a pointer to a tgSphere right now.
  tgSphere* m_pSphere = tgCast::cast<tgSenseable, tgSphere>(m_pSens);
  // Check: if the cast failed, this will return 0.
  // In that case, this tgSphereSensor does not point to a tgSphere!!!
  assert( m_pSphere != 0);
  // Pick out the XYZ position of the center of mass of this sphere.
  
  //ERROR HERE: Drew thinks that this might return the center of mass of
  // the ENTIRE RIGID BODY, not just the sub-component. For example,
  // if this sphere is connected to a rod, then maybe this returns the
  // center of mass of the rod + sphere!!!
  //TO-DO: examine the auto-compounding code, and see if the pointers
  // get mushed together, and see if btRigidBody outputs the combined
  // COM with any other btRigidBodies it's compounded with, or if
  // it truly only returns the COM of the element within the compound.
  btVector3 com = m_pSphere->centerOfMass();
  // Note that the 'orientation' method also returns a btVector3.

  //DEBUGGING.
  // Technically, the COM here is of the whole compound rigid.
  // Let's try to change that.
  // We'll pick out the collision shape from Bullet, which (if it's compound,)
  // should have a list of the sub-collision shapes.
  // TO-DO: can we get the transforms (world positions) of the individual
  // sub-shapes? Was looking into Bullet's library and it seems the child
  // shapes of a compound have transforms. Although... might be local
  // transforms with respect to the local frame?

  //DEBUGGING
  // Pick out the rigid body, then the collision shape, then cast to a compound
  // collision shape, then for each shape, print out some debugging info
  // about it. Do both have the same COM? Origin?
  /*
  std::cout << "Inside tgSphereSensor, pick out the multiple shapes, "
	    << "and return their origins." << std::endl;
  btCollisionShape* wholeShape = m_pSphere->getPRigidBody()->getCollisionShape();
  // Cast to a compound shape
  btCompoundShape* wholeShapeCompound =
    tgCast::cast<btCollisionShape, btCompoundShape>(wholeShape);
  std::cout << "Compound shape has " << wholeShapeCompound->getNumChildShapes()
	    << " child shapes. Each compound has origin: " << std::endl;
  for(int jj=0; jj < wholeShapeCompound->getNumChildShapes(); jj++){
    btCollisionShape* individualShape = wholeShapeCompound->getChildShape(jj);
    std::cout << individualShape->getName() << std::endl;
    
  }
  
  
  //DEBUGGING
  // Let's see what the local supporting vertex (?) is for each shape.

  // HACK BAD: sphere seems to be the first element, cylinder second,
  // in Drew's demo file. This *will* segfault anywhere else.
  //btCylinderShape* cylinder =
  //  tgCast::cast<btCollisionShape, btCylinderShape>(wholeShapeCompound->getChildShape(1));
  //std::cout << "Cylinder support: " << cylinder->localGetSupportingVer
  
  for(int i=0; i < wholeShapeCompound->getNumChildShapes(); i++){
    // Get the rotation matrix (basis?)
    btTransform trans = wholeShapeCompound->getChildTransform(i);
    btMatrix3x3 basis = trans.getBasis();
    //btMatrix3x3& basis = wholeShapeCompound->getChildTransform(i).getBasis();
    //btQuaternion rotationi = wholeShapeCompound->getChildTransform(i).getRotation();
    // Display all its rows. It's 3x3.
    for(int j=0; j<3; j++){
      // For some reason, getColumn returns a btvector3, but getRow returns
      // a btVector3&.
      std::cout << j << ",, ";
      std::cout << basis[j] << ", " << std::endl;
    }
    //std::cout << rotationi.getAxis();
    std::cout << (*trans.getOrigin()) << std::endl;
  }
  */
  
  // The list of sensor data that will be returned:
  std::vector<std::string> sensordata;

  /**
   * The original version of this section of code, which 
   * output one string, looked like:
   * std::stringstream sensordata;
   * sensordata << com[0] << ","
   *	     << com[1] << ","
   *	     << com[2] << ","
   *	     << m_pSphere->mass() << ",";
   */
  
  // The floats (btScalars?) need to be converted to strings
  // via a stringstream.
  std::stringstream ss;

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
  // mass
  ss << m_pSphere->mass();
  sensordata.push_back( ss.str() );
  ss.str("");
  
  return sensordata;
}

//end.
