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
 * @file CombinedSpineControllerRotVertPosition.cpp
 * @brief Implementation of CombinedSpineControllerRotVertPosition.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "CombinedSpineControllerRotVertPosition.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
// This library
#include "core/tgRod.h"
#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h" // for hinge hack
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h" // for hinge hack
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "helpers/FileHelpers.h"
#include <stdexcept>
#include <math.h>

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
CombinedSpineControllerRotVertPosition::CombinedSpineControllerRotVertPosition(double startTime,
						       btVector3 startTorque,
						       double phaseTwoTime,
						       btVector3 phaseTwoTorque,
						       std::string rodHingeTag,
						       btDynamicsWorld* world) :
  m_startTime(startTime),
  m_startTorque(startTorque),
  m_phaseTwoTime(phaseTwoTime),
  m_phaseTwoTorque(phaseTwoTorque),
  m_rodHingeTag(rodHingeTag),
  m_world(world),
  m_timePassed(0.0)
{
  // start time must be greater than or equal to zero
  if( m_startTime < 0.0 ) {
    throw std::invalid_argument("Start time must be greater than or equal to zero.");
  }
  // torques cannot be null
  else if( (m_startTorque == NULL) || (m_phaseTwoTorque == NULL) ) {
    throw std::invalid_argument("A torque is NULL, must supply a torque.");
  }
  // phase two time must be greater than or equal to zero
  if( m_phaseTwoTime < 0.0 ) {
    throw std::invalid_argument("Phase two time must be greater than or equal to zero.");
  }
  // @TODO: what checks to make on tags?
}

/**
 * For this controller, the onSetup method picks out the first rod with 
 * rodHingeTag.
 * Also, as a hack, this onSetup method adds the hinge constraint between the two
 * rods with m_rodHingeTag.
 */
void CombinedSpineControllerRotVertPosition::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up a CombinedSpineControllerRotVertPosition with tag: "
	    << m_rodHingeTag << std::endl;
  // We have an A element and a B element with this tag.
  // In general, Drew is using "A" to be the one that's at the origin in the
  // local frame of the split vertebra.
  std::string rodHingeTagA = m_rodHingeTag + "A";
  std::string rodHingeTagB = m_rodHingeTag + "B";
  // Get what are (hopefully) exactly one rod each from the whole model:
  std::vector<tgRod*> allRodsA = subject.find<tgRod>(rodHingeTagA);
  std::vector<tgRod*> allRodsB = subject.find<tgRod>(rodHingeTagB);

  // Confirm that both these arryas have exactly one element.
  // Make sure this list is not empty:
  if( allRodsA.empty() ) {
    throw std::invalid_argument("No rods found with rodHingeTagA.");
  }
  if( allRodsB.empty() ) {
    throw std::invalid_argument("No rods found with rodHingeTagB.");
  }
  // Now, we know that element 0 exists for each.
  // Confirm that it is not a null pointer.
  if( allRodsA[0] == NULL) {
    throw std::runtime_error("Pointer to rod with rodHingeTagA is NULL.");
  }
  if( allRodsB[0] == NULL) {
    throw std::runtime_error("Pointer to rod with rodHingeTagB is NULL.");
  }

  // We'll need these pointers for applying torques later.
  hingedRodA = allRodsA[0];
  hingedRodB = allRodsB[0];

  // Next, create the rotating ("hinge") joint.
  btRigidBody* rodA_rb = allRodsA[0]->getPRigidBody();
  btRigidBody* rodB_rb = allRodsB[0]->getPRigidBody();
  // We also need to calculate their displacement so we can set the constraint
  // correctly.
  btVector3 rodA_com = allRodsA[0]->centerOfMass();
  btVector3 rodB_com = allRodsB[0]->centerOfMass();
  // If we do A minus B, then the displacement should be the second argument
  // to the hinge constraint.
  btVector3 net_com = rodA_com - rodB_com;
  // Create the hinge constraint
  // Constructor is: 2 x btRigidBody, 4 x btVector3, 1 x bool.
  // For TwoSegSpine: first btVector3 is (-10, 0, 0), or whatever the spacing
  //    between two vertebrae should be.
  // For the rotating joint, need to compensate for the vertical translation,
  // which could be like +30 to rod 2.
  // 5 worked, also 10.
  // Previously was:
  /*
  btHingeConstraint* rotHinge =
    new btHingeConstraint(*rodA_rb, *rodB_rb, btVector3(4, 0, 0),
			  btVector3(0, 0, 0), btVector3(1, 0, 0),
			  btVector3(1, 0, 0), false);
  */
  
  btHingeConstraint* rotHinge =
    new btHingeConstraint(*rodA_rb, *rodB_rb, btVector3(0, 0, 0),
			  net_com, btVector3(1, 0, 0),
			  btVector3(1, 0, 0), false);
  // Add to the world.
  m_world->addConstraint( rotHinge );
  // fingers crossed...
  // NOPE. this segfaults on reset (space bar.) That's probably better for now,
  // actually, until we get this formalized...
}

/**
 * The onStep method does one of the following things:
 * If between time zero and startTime: apply no torque.
 * If between startTime and phaseTwoTime: apply startTorque.
 * If after phaseTwoTime: apply phaseTwoTorque.
 */
void CombinedSpineControllerRotVertPosition::onStep(TensegrityModel& subject, double dt)
{
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, check which action to perform:
  if( m_timePassed > m_startTime ) {
    // Get the rotation matrix that will transform the body-aligned torque
    // with the world-aligned torque. This will allow us to apply a torque
    // only along the axis of the hinge.
    btTransform worldAlignment = hingedRodA->getPRigidBody()->getWorldTransform();
    btMatrix3x3 worldAlignmentBasis = worldAlignment.getBasis();
    // Calculate the first-phase torque, then change it later if needed.
    btVector3 torqueToApply = m_startTorque;
    btVector3 worldAlignedTorque = worldAlignmentBasis * m_startTorque;
    if( m_timePassed > m_phaseTwoTime ) {
      // If time for phase two torque, calculate it:
      torqueToApply = m_phaseTwoTorque;
      worldAlignedTorque = worldAlignmentBasis * torqueToApply;
    }
    // Finally, apply the torque.
    // Note that we apply equal and opposite torques to the two rods.
    //std::cout << "Applying torque: " << worldAlignedTorque << std::endl;
    //std::cout << "Applying torque: " << torqueToApply << std::endl;
    //std::cout << "World alignment is: " << worldAlignmentBasis << std::endl;
    //hingedRodA->getPRigidBody()->applyTorqueImpulse( worldAlignedTorque );
    //hingedRodB->getPRigidBody()->applyTorqueImpulse( -worldAlignedTorque );

    // Let's do a bit of math to find the Euler angles between the two coordinate
    // frames of the vertebra halves. This way, we'll be able to measure the angle
    // of rotation between them (along the axis of the constraint), to do a
    // position controller.
    // Thanks to the wikipedia entry on rotation matrices, and
    // https://math.stackexchange.com/questions/1870661/find-angle-between-two-coordinate-systems

    btVector3 xAxis(1, 0, 0); // since we want to rotate along x
    // Orient the vector along the axis of the first body
    btVector3 xAxisRodA = worldAlignmentBasis * xAxis;
    // DEBUGGING: let's see what this new unit vector will be.
    //std::cout << xAxisRodA << std::endl;
    // great, as expected, it's very close to the (1,0,0) of the world.
    
    // Next, let's see if we can get a rotation matrix that represents
    // a specific rotation around this new unit vector.
    // The angle of rotation around xAxisRodA will be
    btScalar t = 0.3; // local rotation, in radians? Named "t" for "theta."
    // Write out each component of the new rotation matrix, according to
    // https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
    // rename xAxisRodA for brevity
    btVector3 u = xAxisRodA;
    // the std::pow function takes doubles, and I'm getting some issues with
    // Bullet's automatic conversion with btScalar.
    // So, let's use doubles.
    double ux = u.getX();
    double uy = u.getY();
    double uz = u.getZ();
    // Note, pow takes (double, double), so needs to be 2.0 and not just 2
    // First row:
    btScalar r11 = cos(t) + std::pow(ux, 2.0) * ( 1 - cos(t));
    btScalar r12 = ux * uy * (1 - cos(t)) - uz * sin(t);
    btScalar r13 = ux * uz * (1 - cos(t)) + uy * sin(t);
    // Second row:
    btScalar r21 = uy * ux * (1 - cos(t)) + uz * sin(t);
    btScalar r22 = cos(t) + std::pow(uy, 2.0) * (1 - cos(t));
    btScalar r23 = uy * uz * (1 - cos(t)) - ux * sin(t);
    // Third row:
    btScalar r31 = uz * ux * (1 - cos(t)) - uy * sin(t);
    btScalar r32 = uz * uy * (1 - cos(t)) + ux * sin(t);
    btScalar r33 = cos(t) + std::pow(uz, 2.0) * (1 - cos(t));
    // Create the new rotation matrix with these values.
    btMatrix3x3 rodbRot(r11, r12, r13,
			r21, r22, r23,
			r32, r32, r33);
    // DEBUGGING
    //std::cout << rodbRot.getColumn(2) << std::endl;

    // ...finally, let's try and manually set the rotation of rod B using
    // this rotation matrix.
    // Step 1, get the second rod's world transform (position)
    btTransform rodbTransform = hingedRodB->getPRigidBody()->getWorldTransform();
    // get it in quaternion form
    //btQuaternion rodbQuat = rodb
    // Step 2, get the euler angles from the rotation matrix
    btScalar yaw;
    btScalar pitch;
    btScalar roll;
    rodbRot.getEulerYPR(yaw, pitch, roll);
    //DEBUGGING
    //std::cout << yaw << " " << pitch << " " << roll << std::endl;
    // Step 3, apply the rotation to the 
    //rodbTransform.setRotation
    //hingedRodB->getPRigidBody()->setCenterOfMassTransform(rodbTransform);

    // Let's try creating a transform directly from our rotation matrix
    // and setting it.
    btTransform rodbRotTransform = btTransform(rodbRot);
    //hingedRodB->getPRigidBody()->setCenterOfMassTransform(rodbTransformRotated);
    // Interesting! It does seem to put the vertebra along the world-aligned axis.
    // Maybe multiply the two transforms? Which order?
    btTransform rodbTransformRotated = rodbRotTransform * rodbTransform;
    //hingedRodB->getPRigidBody()->setCenterOfMassTransform(rodbTransformRotated);
    // haha. robot flies away.

    // Let's try the original idea and instead of setting the rotation, calculate
    // the difference, and apply a control input.

    // Again, from the wikipedia article, the unit vector of the rotation
    // is [h - f; c - g; d - b], which is then
    // [r32 - r23; r13 - r31; r21 - r12]
    btVector3 uRot( r32 - r23, r13 - r31, r21 - r12);
    //DEBUGGING
    std::cout << uRot << std::endl;
    // kind of looks like the second element of uRot is the angle we want to change.
    // Great. Let's re-do the calculations above.
    // First, a control constant. The angle seems to be in the range of
    // like 0.00 to 0.03 radians, for our purposes. And the torque to apply
    // is on the order of 0.2. So maybe a K of 5 or 10?
    double K = 50; // 50 worked
    // set point:
    double t_set = 0.1; // 0.3 worked
    // Calculate the new torque we want to apply, K * (x - x_ref)
    // we've arbitrarily choosen torques to be negative?
    // torque only goes along the x-axis, like in the original use of this controller
    btVector3 controlledTorque(-K * (uRot.getY() - t_set), 0, 0);
    // torque, aligned with the world
    btVector3 worldControlledTorque = worldAlignmentBasis * controlledTorque;
    // ...apply
    hingedRodA->getPRigidBody()->applyTorqueImpulse( worldControlledTorque );
    hingedRodB->getPRigidBody()->applyTorqueImpulse( -worldControlledTorque );
    // this worked. With just P control, result is a bit shaky. Probably
    // need to put in I and D terms. Or maybe could just tune the controller better.
  }
}
	
 
