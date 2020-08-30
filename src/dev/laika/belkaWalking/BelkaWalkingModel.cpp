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
 * @file BelkaWalkingModel.cpp
 * @brief Contains the implementation of class BelkaWalkingModel
 * @author Andrew P. Sabelhaus
 * $Id$
 */

// This module
#include "BelkaWalkingModel.h"
// This library
#include "core/tgCast.h"
#include "core/tgWorld.h" // for hinge hack
#include "core/tgWorldBulletPhysicsImpl.h" // for hinge hack
#include "core/tgBox.h"
//#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h" // for hinge hack
// #include "BulletDynamics/ConstraintSolver/btHingeConstraint.h" // for hinge hack, now in .h file where it should have been
// The C++ Standard Library
#include <iostream>
#include <stdexcept>
#include <math.h>

// The two constructors. Should just call the parent.
BelkaWalkingModel::BelkaWalkingModel(const std::string& structurePath) :
    TensegrityModel(structurePath) 
{
}

BelkaWalkingModel::BelkaWalkingModel(const std::string& structurePath, bool debugging):
  TensegrityModel(structurePath, debugging)
{
}


/***************************************
 * The primary functions., called from other classes.
 **************************************/


void BelkaWalkingModel::setup(tgWorld& world)
{
  // Call the parent's function.
  std::cout << "Setting up the BelkaWalkingModel using TensegrityModel's YAML parser..." << std::endl;
  TensegrityModel::setup(world);

  // Next, add the btHingeConstraints for the legs.
  // First, the tags for each hinge. Here's the tags for the hips/shoulders:
  std::string hipsTagForHinge = "hipsBack";
  std::string shouldersTagForHinge = "shouldersBack";

  // Tags for each leg's box to attach:
  std::string legBackLeftTagForHinge = "legBoxBackLeft";
  std::string legBackRightTagForHinge = "legBoxBackRight";
  std::string legFrontLeftTagForHinge = "legBoxFrontLeft";
  std::string legFrontRightTagForHinge = "legBoxFrontRight";

  std::cout << "Setting up the btHingeConstraints for the BelkaWalkingModel's legs."
	    << std::endl;

  // Let's get a reference to the btDynamicsWorld. (Hope this is set up by now?)
  // Next, we need to get a reference to the Bullet Physics world, to put the hinges.
  tgWorldImpl& impl = world.implementation();
  tgWorldBulletPhysicsImpl& bulletWorld =
    static_cast<tgWorldBulletPhysicsImpl&>(impl);
  btDynamicsWorld* btWorld = &bulletWorld.dynamicsWorld();

  /**
   * Let's get references to the rods we'll use for the shoulder/hips hinges.
   */

  //
  // Hips:
  //
  
  std::vector<tgRod*> hipHingeRods = find<tgRod>(hipsTagForHinge);
  // Make sure this list is not empty:
  if( hipHingeRods.empty() ) {
    throw std::invalid_argument("No rods found with hipsTagForHinge.");
  }
  // Now, we know that element 0 exists.
  // Confirm that it is not a null pointer.
  if( hipHingeRods[0] == NULL) {
    throw std::runtime_error("Pointer to the first rod with hipsTagForHinge is NULL.");
  }

  // Arbitrarily choose the first of the two rods. Doesn't matter, just off
  // by an orientation.
  btRigidBody* hipHingeRod = hipHingeRods[0]->getPRigidBody();
  
  //
  // Shoulders:
  //

  std::vector<tgRod*> shoulderHingeRods = find<tgRod>(shouldersTagForHinge);
  // Make sure this list is not empty:
  if( shoulderHingeRods.empty() ) {
    throw std::invalid_argument("No rods found with shouldersTagForHinge.");
  }
  // Now, we know that element 0 exists.
  // Confirm that it is not a null pointer.
  if( shoulderHingeRods[0] == NULL) {
    throw std::runtime_error("Pointer to the first rod with shouldersTagForHinge is NULL.");
  }

  // Arbitrarily choose the first of the two rods. Doesn't matter, just off
  // by an orientation.
  btRigidBody* shoulderHingeRod = shoulderHingeRods[0]->getPRigidBody();

  /**
   * Next, let's do the same for each leg.
   */

  //
  // Back Left Leg:
  //
  
  // Note, these are BOXES not rods.
  std::vector<tgBox*> legBackLeftHingeBoxes = find<tgBox>(legBackLeftTagForHinge);
  // Make sure this list is not empty:
  if( legBackLeftHingeBoxes.empty() ) {
    throw std::invalid_argument("No boxes found with legBackLeftTagForHinge.");
  }
  // Now, we know that element 0 exists.
  // Confirm that it is not a null pointer.
  if( legBackLeftHingeBoxes[0] == NULL) {
    throw std::runtime_error("Pointer to the first box with legBackLeftTagForHinge is NULL.");
  }
  // There really should only be one of these.
  btRigidBody* legBackLeftHingeBox = legBackLeftHingeBoxes[0]->getPRigidBody();

  //
  // Back Right Leg:
  //
  
  std::vector<tgBox*> legBackRightHingeBoxes = find<tgBox>(legBackRightTagForHinge);
  // Make sure this list is not empty:
  if( legBackRightHingeBoxes.empty() ) {
    throw std::invalid_argument("No boxes found with legBackRightTagForHinge.");
  }
  // Now, we know that element 0 exists.
  // Confirm that it is not a null pointer.
  if( legBackRightHingeBoxes[0] == NULL) {
    throw std::runtime_error("Pointer to the first box with legBackRightTagForHinge is NULL.");
  }
  // There really should only be one of these.
  btRigidBody* legBackRightHingeBox = legBackRightHingeBoxes[0]->getPRigidBody();

  //
  // Front Left Leg:
  //

  std::vector<tgBox*> legFrontLeftHingeBoxes = find<tgBox>(legFrontLeftTagForHinge);
  // Make sure this list is not empty:
  if( legFrontLeftHingeBoxes.empty() ) {
    throw std::invalid_argument("No boxes found with legFrontLefttTagForHinge.");
  }
  // Now, we know that element 0 exists.
  // Confirm that it is not a null pointer.
  if( legFrontLeftHingeBoxes[0] == NULL) {
    throw std::runtime_error("Pointer to the first box with legfrontleftTagForHinge is NULL.");
  }
  // There really should only be one of these.
  btRigidBody* legFrontLeftHingeBox = legFrontLeftHingeBoxes[0]->getPRigidBody();

  //
  // Front Right Leg:
  //

  std::vector<tgBox*> legFrontRightHingeBoxes = find<tgBox>(legFrontRightTagForHinge);
  // Make sure this list is not empty:
  if( legFrontRightHingeBoxes.empty() ) {
    throw std::invalid_argument("No boxes found with legFrontRightTagForHinge.");
  }
  // Now, we know that element 0 exists.
  // Confirm that it is not a null pointer.
  if( legFrontRightHingeBoxes[0] == NULL) {
    throw std::runtime_error("Pointer to the first box with legFrontRightTagForHinge is NULL.");
  }
  // There really should only be one of these.
  btRigidBody* legFrontRightHingeBox = legFrontRightHingeBoxes[0]->getPRigidBody();

  /**
   * CREATE THE HINGE CONSTRAINTS
   */

  // Constructor is: 2 x btRigidBody, 4 x btVector3, 1 x bool.
  // The frist two btVector3 are location with respect to the center of mass of the body, last two are axis/orientation of the hinge at each of those points.

  // Let's try the following. Assume the frames are with respect to CoM of the body, wherever that is. 
  // The vector from CoM to world point is then
  // p = h - r
  // where h is the vector in the world frame, r is the CoM.
  // So, let's define the desired hinge position of all four shoulders. 
  // First, as created in BelkaWith1DOFLegs.yaml, before the final shift: 19.2cm left/right, 19cm fudge factor into/out of the board
  btVector3 legBLworld = btVector3(19.2, 0.0, -19.0);
  btVector3 legBRworld = btVector3(19.2, 0.0, 19.0);
  btVector3 legFLworld = btVector3(-76.0, 0.0, -19.0);
  btVector3 legFRworld = btVector3(-76.0, 0.0, -19.0);
  // Adjust all vectors in accordance with the robot's final location at the start of the simulation
  btVector3 finaltrans = btVector3(0.0, 29.1, 0.0);
  legBLworld += finaltrans;
  legBRworld += finaltrans;
  legFLworld += finaltrans;
  legFRworld += finaltrans;
  // Now, get the relative vector for each body in each case.
  btVector3 hipHingeRodCoM = hipHingeRod->getCenterOfMassTransform().getOrigin();
  btVector3 shoulderHingeRodCoM = shoulderHingeRod->getCenterOfMassTransform().getOrigin();
  btVector3 legBLHingeBoxCoM = legBackLeftHingeBox->getCenterOfMassTransform().getOrigin();
  btVector3 legBRHingeBoxCoM = legBackRightHingeBox->getCenterOfMassTransform().getOrigin();
  btVector3 legFLHingeBoxCoM = legFrontLeftHingeBox->getCenterOfMassTransform().getOrigin();
  btVector3 legFRHingeBoxCoM = legFrontRightHingeBox->getCenterOfMassTransform().getOrigin();
  // and finally, the eight vectors we need: four constraints, each constraint has two.
  // back left
  btVector3 hipBLrelative = legBLworld - hipHingeRodCoM;
  btVector3 legBLrelative = legBLworld - legBLHingeBoxCoM;
  // back right
  btVector3 hipBRrelative = legBRworld - hipHingeRodCoM;
  btVector3 legBRrelative = legBRworld - legBRHingeBoxCoM;
  // front left
  btVector3 shoulderFLrelative = legFLworld - shoulderHingeRodCoM;
  btVector3 legFLrelative = legFLworld - legFLHingeBoxCoM;
  // front right
  btVector3 shoulderFRrelative = legFRworld - shoulderHingeRodCoM;
  btVector3 legFRrelative = legFRworld - legFRHingeBoxCoM;

  // the bullet demo example calls enableAngularMotor first before adding the constraint to the world...
  btHingeConstraint* legBackLeftHinge =
    new btHingeConstraint(*hipHingeRod, *legBackLeftHingeBox, 
        hipBLrelative,
			  legBLrelative, 
        btVector3(0, 0, 1),
			  btVector3(0, 0, 1));

  // Add the hinge to the world.
  btWorld->addConstraint(legBackLeftHinge);

  // For the back right:
  btHingeConstraint* legBackRightHinge =
    new btHingeConstraint(*hipHingeRod, *legBackRightHingeBox,
			  hipBRrelative,
			  legBRrelative, 
        btVector3(0, 0, 1),
			  btVector3(0, 0, 1));
  btWorld->addConstraint(legBackRightHinge);

  // For the front left:
  btHingeConstraint* legFrontLeftHinge =
    new btHingeConstraint(*shoulderHingeRod, *legFrontLeftHingeBox,
			  shoulderFLrelative,
			  legFLrelative, 
        btVector3(0, 0, 1),
			  btVector3(0, 0, 1));
  btWorld->addConstraint(legFrontLeftHinge);

  // For the front right:
  btHingeConstraint* legFrontRightHinge =
    new btHingeConstraint(*shoulderHingeRod, *legFrontRightHingeBox,
			  shoulderFRrelative,
			  legFRrelative, 
        btVector3(0, 0, 1),
			  btVector3(0, 0, 1));
  btWorld->addConstraint(legFrontRightHinge);

  // Finally, store all the hinge pointers for future use.
  legHinges.push_back(legBackLeftHinge);
  legHinges.push_back(legBackRightHinge);
  legHinges.push_back(legFrontLeftHinge);
  legHinges.push_back(legFrontRightHinge);

  // Initialize our control inputs to zero. That means leg angle of zero (i.e. perp to ground), and 0% retraction for bending/rotation cables.
  // I'm still unclear as to what version of C++ we're using, so just to be super backward compatible, 
  // here's an ugly loop. There are 6 inputs.
  double initial_angle = 0.0;
  u_in.clear();
  for(size_t i=0; i < 4; i++){
    u_in.push_back(initial_angle);
  }
  // tack on the two retractions.
  u_in.push_back(0.0); 
  u_in.push_back(0.0);
}

std::vector<btHingeConstraint*> BelkaWalkingModel::getLegHinges(){
  return legHinges;
}

void BelkaWalkingModel::keyboardCallback(unsigned char key, int x, int y)
{
  // DEBUGGING
  // std::cout << "Caught key press " << key << " in BelkaWalkingModel." << std::endl;
  // though it's kind redundant, cleaner if the helper does the job
  updateU(key);
}

void BelkaWalkingModel::updateU(unsigned char key)
{
  // We have 6 control inputs, and two keys each to adjust them +/-. Twelve keys.
  // Also, since the parents use up most of the lower-case keys, all upper-case here.
  // Key map:
  // -------------
  // W E R T   U I
  // S D F G   J K
  // -------------
  // W = LegA+
  // S = LegA-
  // E = LegB+
  // ...
  // G = LegD-
  // U = bend right
  // J = bend left
  // I = CCW spine
  // K = CW spine

  // NOTE: all the angles have to be constrained between -180, 180.
  switch (key)
  {
  case 'W':
    u_in[0] += ang_incr;
    u_in[0] = adjTheta(u_in[0]);
    std::cout << "LegA theta = " << u_in[0] << std::endl;
    break;
  
  case 'S':
    u_in[0] -= ang_incr;
    u_in[0] = adjTheta(u_in[0]);
    std::cout << "LegA theta = " << u_in[0] << std::endl;
    break;
  
  case 'E':
    u_in[1] += ang_incr;
    u_in[1] = adjTheta(u_in[1]);
    std::cout << "LegB theta = " << u_in[1] << std::endl;
    break;
  
  case 'D':
    u_in[1] -= ang_incr;
    u_in[1] = adjTheta(u_in[1]);
    std::cout << "LegB theta = " << u_in[1] << std::endl;
    break;
  
  case 'R':
    u_in[2] += ang_incr;
    u_in[2] = adjTheta(u_in[2]);
    std::cout << "LegC theta = " << u_in[2] << std::endl;
    break;
  
  case 'F':
    u_in[2] -= ang_incr;
    u_in[2] = adjTheta(u_in[2]);
    std::cout << "LegC theta = " << u_in[2] << std::endl;
    break;
  
  case 'T':
    u_in[3] += ang_incr;
    u_in[3] = adjTheta(u_in[3]);
    std::cout << "LegD theta = " << u_in[3] << std::endl;
    break;
  
  case 'G':
    u_in[3] -= ang_incr;
    u_in[3] = adjTheta(u_in[3]);
    std::cout << "LegD theta = " << u_in[3] << std::endl;
    break;
  
  case 'U':
    u_in[4] += cbl_incr;
    std::cout << "Left/Right % = " << u_in[4] << std::endl;
    break;
  
  case 'J':
    u_in[4] -= cbl_incr;
    std::cout << "Left/Right % = " << u_in[4] << std::endl;
    break;
  
  case 'I':
    u_in[5] += cbl_incr;
    std::cout << "CCW/CW % = " << u_in[5] << std::endl;
    break;
  
  case 'K':
    u_in[5] -= cbl_incr;
    std::cout << "CCW/CW % = " << u_in[5] << std::endl;
    break;
  
  default:
    break;
  }
}

double BelkaWalkingModel::adjTheta(double theta)
{
  if( theta > 180.0 )
  {
    theta -= 360;
  }
  else if( theta < -180.0 )
  {
    theta += 360;
  }
  return theta;
}

// TO-DO: add teardown method and properly recreate the hinges.