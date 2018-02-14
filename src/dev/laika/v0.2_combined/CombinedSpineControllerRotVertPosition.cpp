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
						       double setAngle,
						       std::string rodHingeTag,
						       btDynamicsWorld* world) :
  m_startTime(startTime),
  m_setAngle(setAngle),
  m_rodHingeTag(rodHingeTag),
  m_world(world),
  m_timePassed(0.0),
  m_accumulatedError(0.0),
  m_prevError(0.0)
{
  // start time must be greater than or equal to zero
  if( m_startTime < 0.0 ) {
    throw std::invalid_argument("Start time must be greater than or equal to zero.");
  }
  // set angle cannot be null
  else if( (m_setAngle == NULL )) {
    throw std::invalid_argument("Set angle is NULL, need to specify angle.");
  }
  // @TODO: what checks to make on tags?
}

/**
 * For this controller, the onSetup method picks out the two rods with rodHingeTag
 * and "A" or "B" attached to the tag. The hingeconstraint is created here,
 * which is not great, but makes it so we don't need to make an extra model class.
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
  btHingeConstraint* rotHinge =
    new btHingeConstraint(*rodA_rb, *rodB_rb, btVector3(0, 0, 0),
			  net_com, btVector3(1, 0, 0),
			  btVector3(1, 0, 0), false);
  // Add to the world.
  m_world->addConstraint( rotHinge );
  // This segfaults on reset (space bar.) That's probably better for now,
  // actually, until we get this formalized...
}

/**
 * The onStep method does one of the following things:
 * If between time zero and startTime: apply no torque.
 * If after startTime: apply a control to track the desired position (set angle).
 */
void CombinedSpineControllerRotVertPosition::onStep(TensegrityModel& subject, double dt)
{
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, check which action to perform:
  if( m_timePassed > m_startTime ) {
    // Calculate and apply a torque that will be controlled around a position
    // (angle between the two vertebrae.)
    
    // We want to obtain the rotation around the axis that aligns rod A and rod B.
    // Luckily enough, we already know that the frame will be aligned along the
    // X-axis, since that's how we've set up the btHingConstraint.
    // So, we can take advantage of quaternions here:
    // a quaternion is (axis + angle), so just taking the angle of the quaternion
    // that represents this A-to-B rotation will give us what we want.

    // Here's the procedure that is used in this code.
    // We'll use rotation matrices first.
    // 1) Get the rotation matrix of each of the two frames (rod A, rod B), with
    //    respect to the world
    // 2) Get the net rotation from one frame to another
    // 2) Get the angle along the axis that connects these two frames (which, if R is going from A to B, just the rotation implied in R around R's unit vector.)
    // (note, we *know* that the unit vector u of R_net will really only have one component, along the x-direction, since we've constrainted along the other two.
    // 3) Calculate the difference between this angle, and the t_set
    // 4) Calculate and apply a torque proportional to t_diff (the control.)
    
    // Much credit to the wikipedia article on rotations:
    // https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
    // As well as some comments on coordinate systems from stackexchange:
    // https://math.stackexchange.com/questions/1870661/find-angle-between-two-coordinate-systems  
    
    // OK, so one way to get the net rotation matrix between the two is to
    // first rotate A back to the origin, then apply the rotation to get from
    // the origin to B. Since we have R_a_origin, and R_b_origin, given by the basis
    // for the transformation for both rigid body objects, this could be:
    btTransform rodaTransform = hingedRodA->getPRigidBody()->getWorldTransform();
    btTransform rodbTransform = hingedRodB->getPRigidBody()->getWorldTransform();
    // We can get the rotation matrices from each transform
    // (note that we want to ignore the translation!)
    btMatrix3x3 rodaWorldRotation = rodaTransform.getBasis();
    btMatrix3x3 rodbWorldRotation = rodbTransform.getBasis();
    // ...we might have been able to just use quaternions here, but this made more
    // sense to Drew at the time.
    // To get the net, invert the rod A transform, then apply the B transform.
    // Using Bullet's methods, we can say:
    btMatrix3x3 netRotation = rodaWorldRotation.inverse() * rodbWorldRotation;
    // The order doesn't matter here, just off by a +/-, since the only difference
    // when "looking" from one rod to another is the direction of the unit vector.

    // Next, let's get the quaternion for this net rotation, and extract its angle.
    btQuaternion netRotQuat;
    netRotation.getRotation(netRotQuat);
    btScalar netRotScalar = netRotQuat.getAngle();
    // for checking: let's get the axis here too. Should be only in one direction.
    //btVector3 netRotAxis = netRotQuat.getAxis();
    //DEBUGGING
    //std::cout << "netRotScalar: " << netRotScalar << std::endl;
    //std::cout << "netRotAxis: " << netRotAxis << std::endl;
    // GREAT THIS SEEMS TO WORK. netRotAxis is roughly (1, 0, 0),
    // and netRotScalar is some reasonable value.
    
    // Great. Let's perform the control.
    // First, a control constant. The angle seems to be in the range of
    // like 0.00 to 0.03 radians, for our purposes. And the torque to apply
    // is on the order of 0.2. So maybe a K of 5 or 10?
    double K = 1000; // 50 worked, but lots of overshoot
    // Calculate the new torque we want to apply, - K * (x - x_ref)
    // we've arbitrarily choosen torques to be negative?
    // The error between the current and desired, x - x_ref, is
    double error = netRotScalar - m_setAngle;
    // As a torque vector, along the x-axis:
    btVector3 controlledTorque( -K * error, 0, 0);
    //DEBUGGING: what's the error that we're controlling around?
    // Need to develop a control such that this trends to zero.
    std::cout << "error: " << error << std::endl;
    // it would be interesting to plot this.
    // torque, aligned with the world, referenced against rod A.
    // (choice is arbitrary, since we can just do +/- switching A to B.)
    btVector3 worldControlledTorque = rodaWorldRotation * controlledTorque;
    // ...apply the torque, opposite to each body.
    hingedRodA->getPRigidBody()->applyTorqueImpulse( worldControlledTorque );
    hingedRodB->getPRigidBody()->applyTorqueImpulse( -worldControlledTorque );
    // this worked. With just P control, seems to have steady-state error.
    // Need to add I.
  }
}
	
 
