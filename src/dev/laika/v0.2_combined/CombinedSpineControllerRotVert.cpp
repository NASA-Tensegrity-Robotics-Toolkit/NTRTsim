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
 * @file CombinedSpineControllerRotVert.cpp
 * @brief Implementation of CombinedSpineControllerRotVert.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "CombinedSpineControllerRotVert.h"
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

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
CombinedSpineControllerRotVert::CombinedSpineControllerRotVert(double startTime,
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
void CombinedSpineControllerRotVert::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up a CombinedSpineControllerRotVert with tag: "
	    << m_rodHingeTag << std::endl;
  // We have an A element and a B element with this tag.
  // In general, Drew is using "A" to be the one that's at the origin in the
  // local frame of the split vertebra.
  // First, get all the tgRods.
  std::vector<tgRod*> allRods = subject.find<tgRod>(m_rodHingeTag);
  // Make sure this list is not empty:
  if( allRods.empty() ) {
    throw std::invalid_argument("No rods found with rodHingedTag.");
  }
  // Now, we know that element 0 exists.
  // Confirm that it is not a null pointer.
  if( allRods[0] == NULL) {
    throw std::runtime_error("Pointer to the first rod with rodHingeTag is NULL.");
  }
  if( allRods[1] == NULL) {
    throw std::runtime_error("Pointer to the second rod with rodHingeTag is NULL.");
  }
  // Finally, store these pointers.
  // This is so we can apply one torque to one side,
  // and another torque to another side.
  hingedRodA = allRods[0];
  hingedRodB = allRods[1];

  std::cout << "Size of allRods: "
	    << allRods.size() << std::endl;

  // Next, create the rotating ("hinge") joint.
  btRigidBody* rodA_rb = allRods[0]->getPRigidBody();
  btRigidBody* rodB_rb = allRods[1]->getPRigidBody();
  // Create the hinge constraint
  // Constructor is: 2 x btRigidBody, 4 x btVector3, 1 x bool.
  // For TwoSegSpine: first btVector3 is (-10, 0, 0), or whatever the spacing
  //    between two vertebrae should be.
  // For the rotating joint, need to compensate for the vertical translation,
  // which could be like +30 to rod 2.
  // 5 worked, also 10.
  btHingeConstraint* rotHinge =
    new btHingeConstraint(*rodA_rb, *rodB_rb, btVector3(4, 0, 0),
			  btVector3(0, 0, 0), btVector3(1, 0, 0),
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
void CombinedSpineControllerRotVert::onStep(TensegrityModel& subject, double dt)
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
      worldAlignedTorque = worldAlignmentBasis * m_phaseTwoTorque;
    }
    // Finally, apply the torque.
    // Note that we apply equal and opposite torques to the two rods.
    //std::cout << "Applying torque: " << worldAlignedTorque << std::endl;
    //std::cout << "Applying torque: " << torqueToApply << std::endl;
    //std::cout << "World alignment is: " << worldAlignmentBasis << std::endl;
    hingedRodA->getPRigidBody()->applyTorqueImpulse( worldAlignedTorque );
    hingedRodB->getPRigidBody()->applyTorqueImpulse( -worldAlignedTorque );
  }
}
	
 
