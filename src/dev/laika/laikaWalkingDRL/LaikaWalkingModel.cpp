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
 * @file LaikaWalkingModel.cpp
 * @brief Contains the implementation of class LaikaWalkingModel
 * @author Andrew P. Sabelhaus
 * $Id$
 */

// This module
#include "LaikaWalkingModel.h"
// This library
#include "core/tgCast.h"
#include "core/tgWorld.h" // for hinge hack
#include "core/tgWorldBulletPhysicsImpl.h" // for hinge hack
#include "core/tgBox.h"
//#include "core/tgBaseRigid
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
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h" // for hinge hack
// The C++ Standard Library
#include <iostream>
#include <stdexcept>
#include <math.h>
#include <sstream>

// The two constructors. Should just call the parent.
LaikaWalkingModel::LaikaWalkingModel(const std::string& structurePath) :
    TensegrityModel(structurePath)
{
}

LaikaWalkingModel::LaikaWalkingModel(const std::string& structurePath, bool debugging):
  TensegrityModel(structurePath, debugging)
{
}


/*
void LaikaWalkingModel::mapMuscles(LaikaWalkingModel::MuscleMap& muscleMap,
            tgModel& model, size_t segmentCount)
{
    // create names for muscles (for getMuscles function)

    // vertical muscles
    muscleMap["vertical a"] = model.find<tgSpringCableActuator>("vertical muscle a");
    muscleMap["vertical b"] = model.find<tgSpringCableActuator>("vertical muscle b");
    muscleMap["vertical c"] = model.find<tgSpringCableActuator>("vertical muscle c");
    muscleMap["vertical d"] = model.find<tgSpringCableActuator>("vertical muscle d");

    // saddle muscles
    for (size_t i = 1; i < segmentCount ; i++)
    {
        muscleMap[tgString("saddle", i-1)] = model.find<tgSpringCableActuator>(tgString("saddle muscle seg", i-1));

    }
}
*/


/***************************************
 * The primary functions., called from other classes.
 **************************************/


void LaikaWalkingModel::setup(tgWorld& world)
{
  // Call the parent's function.
  std::cout << "Setting up the LaikaWalkingModel using TensegrityModel's YAML parser..." << std::endl;
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

  std::cout << "Setting up the btHingeConstraints for the LaikaWalkingModel's legs."
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

  btHingeConstraint* legBackLeftHinge =
    new btHingeConstraint(*hipHingeRod, *legBackLeftHingeBox,
        btVector3(3, 0, 20),
			  btVector3(0, 16.5, 0), btVector3(0, 0, 1),
			  btVector3(0, 0, 1));
  // 1.5, was hips 2
  // Add the hinge to the world.
  btWorld->addConstraint(legBackLeftHinge);

  // For the back right:
  btHingeConstraint* legBackRightHinge =
    new btHingeConstraint(*hipHingeRod, *legBackRightHingeBox,
			  btVector3(3, 0, -20),
			  btVector3(0, 16.5, 0), btVector3(0, 0, 1),
			  btVector3(0, 0, 1));
  btWorld->addConstraint(legBackRightHinge);

  // For the front left:
  btHingeConstraint* legFrontLeftHinge =
    new btHingeConstraint(*shoulderHingeRod, *legFrontLeftHingeBox,
			  btVector3(-3, 0, 20),
			  btVector3(0, 16.5, 0), btVector3(0, 0, 1),
			  btVector3(0, 0, 1));
  btWorld->addConstraint(legFrontLeftHinge);

  // For the front right:
  btHingeConstraint* legFrontRightHinge =
    new btHingeConstraint(*shoulderHingeRod, *legFrontRightHingeBox,
			  btVector3(-3, 0, -20),
			  btVector3(0, 16.5, 0), btVector3(0, 0, 1),
			  btVector3(0, 0, 1));
  btWorld->addConstraint(legFrontRightHinge);

  std::vector<std::string> actuatorTags;
	actuatorTags.push_back("HF");
  actuatorTags.push_back("HR");
  actuatorTags.push_back("HL");
  actuatorTags.push_back("HB");
  actuatorTags.push_back("SFR");
  actuatorTags.push_back("SRL");
  actuatorTags.push_back("SBF");
  actuatorTags.push_back("SBL");

  m_allActuators.clear();
  m_allActuators = getAllActuators(actuatorTags);
}

/**
 * Method that returns the full state of the robot (all the rigid bodies.)
 * Hard-coded to only select out certain rigid bodies, must be used with
 * the individual vertebrae in the YAML file as well as the hinged stick legs.
 * @return a 108-vector (since 9 rigid bodies x 12 states.)
 */
std::vector<double> LaikaWalkingModel::getLaikaWalkingModelStates()
{
  // We'll be putting the data here:
  std::vector<double> states;

  // First, a list of all the tags we'll be picking out from the children.
  // In order, we want to do shoulder, vertebrae, hips, legs.
  // Pick out the box for each of the shoulders/hips and the boxes for the legs,
  // and the bottom rod of each of the three floating vertebrae.
  std::vector<std::string> laikaRigidBodyTags;
  laikaRigidBodyTags.push_back("shouldersBase");
  laikaRigidBodyTags.push_back("vertebraAbottomrod");
  laikaRigidBodyTags.push_back("vertebraBbottomrod");
  laikaRigidBodyTags.push_back("vertebraCbottomrod");
  laikaRigidBodyTags.push_back("hipsBase");
  laikaRigidBodyTags.push_back("legBoxBackLeft");
  laikaRigidBodyTags.push_back("legBoxBackRight");
  laikaRigidBodyTags.push_back("legBoxFrontLeft");
  laikaRigidBodyTags.push_back("legBoxFrontRight");

  // For each of the tags, do the following.
  // (1) get all the rigid bodies that have that tag
  // (2) confirm that there is exactly one element (one rigid)
  // (3) get the btRigidBody
  // (4) get the positions, orientations, velocities, and rot velocities
  // (5) append each of those to 'states'

  for(int i=0; i < laikaRigidBodyTags.size(); i++) {

    // (1) get the rigid bodies with this tag
    std::vector<tgBaseRigid*> currentBodies =
      find<tgBaseRigid>(laikaRigidBodyTags[i]);
    // Make sure this list is not empty:
    if( currentBodies.size() != 1 ) {
      throw std::invalid_argument("Wrong number of bodies with tag for states.");
    }
    // Now, we know that element 0 exists.
    // (2) Confirm that it is not a null pointer.
    if( currentBodies[0] == NULL) {
      throw std::runtime_error("Pointer to the first rigid body for states is NULL");
    }
    // (3)Get the single body.
    btRigidBody* currentBody = currentBodies[0]->getPRigidBody();

    // (4) In order, get and append positions, orient, vel, and rot vel
    btVector3 pos = currentBody->getCenterOfMassPosition();
    double yaw;
    double pitch;
    double roll;
    currentBody->getCenterOfMassTransform().getBasis().getEulerYPR(yaw, pitch, roll);
    btVector3 vel = currentBody->getLinearVelocity();
    btVector3 angularvel = currentBody->getAngularVelocity();

    // (5) put all this nice data into the 'states' vector.
    // Indexing into a btVector3 happens via the x, y, z methods. Elements 0, 1, 2.
    states.push_back(pos.x());
    states.push_back(pos.y());
    states.push_back(pos.z());
    states.push_back(yaw);
    states.push_back(pitch);
    states.push_back(roll);
    states.push_back(vel.x());
    states.push_back(vel.y());
    states.push_back(vel.z());
    states.push_back(angularvel.x());
    states.push_back(angularvel.y());
    states.push_back(angularvel.z());
  }

  return states;
}

std::vector<double> LaikaWalkingModel::getLaikaWalkingModelCableRL()
{
  std::vector<double> cableRL;
  for (int i = 0; i < m_allActuators.size(); i++) {
    cableRL.push_back(m_allActuators[i]->getRestLength());
  }
  return cableRL;
}

std::vector<tgBasicActuator*> LaikaWalkingModel::getAllActuators(std::vector<std::string> actuatorTags)
{
  std::vector<tgBasicActuator*> allActuators;

  for (int i = 0; i < actuatorTags.size(); i++) {
    // Sort through actuators to make sure the order is the same
    for (int j = 0; j < numVertebrae-1; j++) {
      std::ostringstream num1;
      std::ostringstream num2;
      num1 << j+1;
      num2 << j+2;

      std::string tag;
      tag = actuatorTags[i] + " t" + num1.str() + "/t" + num2.str();
      std::vector<tgBasicActuator*> actuator = find<tgBasicActuator>(tag);
      // std::cout << tag << std::endl;

      // Make sure this list is not empty:
      if(actuator.empty()) {
        throw std::invalid_argument("No actuators found with " + actuatorTags[i] + ".");
      }
      // Now, we know that element 0 exists.
      // Confirm that it is not a null pointer.
      if(actuator[0] == NULL) {
        throw std::runtime_error("Pointer to the first actuator with " + actuatorTags[i] + " is NULL.");
      }
      allActuators.push_back(actuator[0]);
    }
  }

  return allActuators;
}
