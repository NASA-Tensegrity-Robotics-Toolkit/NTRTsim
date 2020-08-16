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
  // For TwoSegSpine: first btVector3 is (-10, 0, 0), or whatever the spacing
  //    between two vertebrae should be.
  // For the rotating joint, need to compensate for the vertical translation,
  // which could be like +30 to rod 2.
  // I think the first two btVectors are the locations of the contact point, relative
  // to each rigid body. Let's do it like an offset from the leg, and zero from
  // the hip. But we need to 
  // The last two btVector3s are the axis for each element.
  // We'll choose to be Y for both.
  // For example - the first btVector3 moves the point on the hips to the edge of
  // the hips, then translates it in by half the rod radius (3/2) to center it.
  // The second btVector3 moves the point on the leg to its top. The total leg
  // height is (30 + sphere radius / 2) = 16.5 ?
  // and then also centers it. (Width = 3.)

  // NOT PLACED ALONG CORRECT POINT? It seems like the end nodes align now, but the
  // center of rotation seems to be slightly "down" the leg...
  // maybe move the point of contact in the Z direction for the hip? Not 1.5 but 3?
  // btHingeConstraint* legBackLeftHinge =
  //   new btHingeConstraint(*hipHingeRod, *legBackLeftHingeBox, btVector3(1.5, 2, -20),
	// 		  btVector3(0, 16.5, 0), btVector3(0, 0, 1),
	// 		  btVector3(0, 0, 1));

  // For the 2020 work: what *should* these be?
  // Hip: we're getting the rod that's the top of the "T"
  // Kinda works: btVector3(3, 3, -19), btVector3(-1.5, 16.5, 0), 
  // The E1 direction (into/out of the board), controlled by (~, ~, x3), is good at 17.8 + ((1/2)*0.5 legbox) + fudge = 19
  // The frame is frustratingly offset. Leg seems OK at 0,0, but the rod seems to give us the corner not the center????
  // Also works: seems clear that the origin is some weird CoM thing. btVector3(0, 0, -19), btVector3(0, 16.5, 0), 

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
}

std::vector<btHingeConstraint*> BelkaWalkingModel::getLegHinges(){
  return legHinges;
}