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
 * @file InvKinTestController.cpp
 * @brief Implementation of InvKinTestController.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "InvKinTestController.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
// This library
//#include "core/tgRod.h"
#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
// #include "BulletDynamics/Dynamics/btDynamicsWorld.h" // for hinge hack
// #include "BulletDynamics/ConstraintSolver/btHingeConstraint.h" // for hinge hack
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
InvKinTestController::InvKinTestController(double startTime,
                  double holdTime, std::string invkinCSVPath) :
  m_startTime(startTime),
  m_holdTime(holdTime),
  m_invkinCSVPath(invkinCSVPath),
  m_timePassed(0.0)
{
  // start time must be greater than or equal to zero
  if( m_startTime < 0.0 ) {
    throw std::invalid_argument("Start time must be greater than or equal to zero.");
  }
  // hold time must be greater than or equal to start time
  if( m_holdTime < m_startTime ) {
    throw std::invalid_argument("Hold time must be greater than or equal to start time.");
  }
  // The CSV file needs to not be null, or an empty string.
  if( m_invkinCSVPath.empty() ){
    throw std::invalid_argument("CSV path not passed in. Must provide a file with inv kin rest lengths.");
  }
  // @TODO: what asserts?
  // Note that unlike in C, we don't need to allocate the maps or vectors.
  // Declaring them creates empty ones for us.
}

/**
 * For this controller, the onSetup method does:
 * 
 */
void InvKinTestController::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up a InvKinTestController with CSV file: "
	    << m_invkinCSVPath << std::endl;
  // call the helpers to do the following: 
  // (a) get the list of tags and store the rest length inputs in their map
  // (b) assign the map of cables.
  // InvKinTestController::assignCableInputMap(subject);
  // InvKinTestController::assignCableTagMap(subject);

  // Do some checks:
  // (1) neither result is empty
  // (2) there are no null pointers in either map
}

/**
 * The onStep method does one of the following things:
 * If between time zero and startTime: no change to rest length (the defaults from the YAML file are used.)
 * If between startTime and holdTime: apply the first rest length from the map ("let settle to initial pose")
 * If after holdTime: apply the rest length corresponding to the appropriate entry in the map
 */
void InvKinTestController::onStep(TensegrityModel& subject, double dt)
{
  // First, increment the accumulator variable.
  m_timePassed += dt;
  // Then, check which action to perform:
  if( m_timePassed > m_startTime ) {
    if(m_timePassed > m_holdTime) {
      // Apply the control input at the point

    }
    else {
      // Apply the first control input
    }
  }
  // else, do nothing.
}
	
 
