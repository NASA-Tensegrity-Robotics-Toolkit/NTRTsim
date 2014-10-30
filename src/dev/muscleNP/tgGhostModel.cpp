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
 * @file tgGhostModel.cpp
 * @brief Contains the definitions of members of class tgRod
 * @author Brian Mirletz and Ryan Adams
 * $Id$
 */

// This module
#include "tgGhostModel.h"
#include "core/tgModelVisitor.h"
// The Bullet Physics library
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>

tgGhostModel::tgGhostModel(btPairCachingGhostObject* pCollisionObject, 
                const tgTags& tags) : 
  tgModel(tags),
  m_pCollisionObject(pCollisionObject)
{
    if (pCollisionObject == NULL)
    {
            throw std::invalid_argument("Pointer to ghostObject is NULL");
    }

    // Postcondition
    assert(invariant());
    assert(m_pCollisionObject == pCollisionObject);
}

tgGhostModel::~tgGhostModel() { }

void tgGhostModel::onVisit(const tgModelVisitor& v) const
{
    v.render(*this);
    
}

void tgGhostModel::teardown()
{
  // This does not appear to be called

  // Postcondition
  // This does not preserve the invariant
}

bool tgGhostModel::invariant() const
{
  return
    (m_pCollisionObject != NULL);
}
