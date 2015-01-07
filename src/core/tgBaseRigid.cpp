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
 * @file tgBaseRigid.cpp
 * @brief Contains the definitions of members of class tgRod
 * @author Brian Mirletz and Ryan Adams
 * $Id$
 */

// This module
#include "tgBaseRigid.h"
#include "tgModelVisitor.h"
// The Bullet Physics library
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>

tgBaseRigid::tgBaseRigid(btRigidBody* pRigidBody, 
                const tgTags& tags) : 
  tgModel(tags),
  m_pRigidBody(pRigidBody),
  m_mass((m_pRigidBody->getInvMass() > 0.0) ? 
     1.0 / (m_pRigidBody->getInvMass()) :
     0.0) // The object is static
{
    if (pRigidBody == NULL)
    {
            throw std::invalid_argument("Pointer to btRigidBody is NULL");
    }

    // Postcondition
    assert(invariant());
    assert(m_pRigidBody == pRigidBody);
    
    // Supress compiler warning for bullet's unused variable
    (void) btInfinityMask;
}

tgBaseRigid::~tgBaseRigid() { }

void tgBaseRigid::onVisit(const tgModelVisitor& v) const
{
    v.render(*this);
    
}

void tgBaseRigid::teardown()
{
  // World owns this
    m_pRigidBody = NULL;
    tgModel::teardown();
  // Postcondition
  // This does not preserve the invariant
}

btVector3 tgBaseRigid::centerOfMass() const
{
  // Precondition
  assert(m_pRigidBody->getMotionState() != NULL);

  btTransform transform;
  m_pRigidBody->getMotionState()->getWorldTransform(transform);
  const btVector3& result = transform.getOrigin();
   
  // Return a copy
  return result;
}

btVector3 tgBaseRigid::orientation() const
{
  //Precondition
  assert(invariant());

  // get the orientation of this rod w.r.t. the world's refernce coorinate system
  // oddly enough, there isn't a getEuler method for btQuaternion, which is
  // returned by getOrientation from RigidBody, so convert it
  // into a rotation matrix first.
  btMatrix3x3 rot = btMatrix3x3(m_pRigidBody->getOrientation());
  btScalar yaw = 0.0;
  btScalar pitch = 0.0;
  btScalar roll = 0.0;
  rot.getEulerYPR(yaw, pitch, roll);
  btVector3 *result = new btVector3(yaw, pitch, roll);
  return *result;
}

bool tgBaseRigid::invariant() const
{
  return
    (m_pRigidBody != NULL) &&
    (m_mass >= 0.0);
}
