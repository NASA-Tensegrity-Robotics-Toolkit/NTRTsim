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
 * @file tgBoxMoreAnchors.cpp
 * @brief Contains the definitions of members of class tgBoxMoreAnchors
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "tgBoxMoreAnchors.h"
#include "core/tgModelVisitor.h"
// The Bullet Physics library
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>

/**
 * The constructor for this class will just call the constructor for the parent.
 * However, we also need to assign m_length for this class too.
 * See the .h file for a to-do about this use of private m_length.
 */
tgBoxMoreAnchors::tgBoxMoreAnchors(btRigidBody* pRigidBody, 
                const tgTags& tags,
                const double length) : 
  tgBox(pRigidBody, tags, length),
  m_length(length)
{
  // This is checked in tgBox too, but check again here so that the
  // error makes more sense if it's ever thrown.
  if (pRigidBody == NULL)
    {
      throw std::invalid_argument("Pointer to btRigidBody is NULL");
    }

  // Postcondition
  assert(invariant());
}

// Destructor does nothing.
tgBoxMoreAnchors::~tgBoxMoreAnchors() { }

// If the tgBoxMoreAnchors ever needed to be rendered differently
// than tgBox, this method allows that to happen. (I think?)
void tgBoxMoreAnchors::onVisit(const tgModelVisitor& v) const
{
    v.render(*this);
    // Do we need to render the base class?
}

bool tgBoxMoreAnchors::invariant() const
{
  return
    (m_length >= 0.0);
}
