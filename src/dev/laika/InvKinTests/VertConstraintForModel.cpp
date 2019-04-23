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
 * @file VertConstraintForModel.cpp
 * @brief Implementation of VertConstraintForModel.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "VertConstraintForModel.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
// This library
#include "core/tgBaseRigid.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <stdexcept>

// Constructor assigns variables, does some simple sanity checks.
VertConstraintForModel::VertConstraintForModel(std::vector<std::string> tags) :
  m_tags(tags)
{
  // can't be an empty list.
  if( tags.empty() ) {
    throw std::invalid_argument("Cannot add vertical constraint to nothing, please pass in a non-empty list of tags.");
  }
  // to-do: better checks on each element of the list.
}

// For the single-tag constructor, it's easiest to make a list and put it
// in as one element. More flexible though not as pretty.
VertConstraintForModel::VertConstraintForModel(std::string tag) {
  // check: can't be an empty string.
  if( tag.empty() ) {
    throw std::invalid_argument("Cannot add a constraint, tag is empty. Pass in nonempty tag.");
  }
  // Record the tag. Make a list:
  std::vector<std::string> tags;
  // add the one tag
  tags.push_back(tag);
  // and assign it to the private variable.
  m_tags = tags;
}

/**
 * For this controller, the onSetup method does:
 * 
 */
void VertConstraintForModel::onSetup(TensegrityModel& subject)
{
  // Iterate through the tags and assign the linear factor.
  for(std::vector<std::string>::iterator it = m_tags.begin(); it != m_tags.end(); ++it) {
    // Call the helper for this tag.
    addVerticalConstraint(subject, *it);
  }
}

/**
 * The onStep method does nothing. We only care about onSetup.
 */
void VertConstraintForModel::onStep(TensegrityModel& subject, double dt)
{
  // nothing
}

// The vertical constraint.
void VertConstraintForModel::addVerticalConstraint(TensegrityModel& subject, std::string tag) {
  // Get the bodies with these tags.
  // Must be base rigids so that mass is a thing.
  std::vector<tgBaseRigid*> foundBodies = subject.find<tgBaseRigid>(tag);
  // For all these, add the vertical "constraint."
  for(std::vector<tgBaseRigid*>::iterator it = foundBodies.begin(); it != foundBodies.end(); ++it) {
    tgBaseRigid* currentBody = *it;
    //debugging
    //std::cout << "Found a body with tags " << currentBody->getTags() << std::endl;
    // Eliminate the ability to move in the 3rd coordinate.
    currentBody->getPRigidBody()->setLinearFactor(btVector3(1,1,0));
  }
}

 
