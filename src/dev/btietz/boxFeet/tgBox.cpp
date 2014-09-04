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
 * @file tgRod.cpp
 * @brief Contains the definitions of members of class tgRod
 * @author Ryan Adams
 * $Id$
 */

// This module
#include "tgBox.h"
#include "tgModelVisitor.h"
// The Bullet Physics library
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>

tgBox::Config::Config(double w, double h, double d,
                        double f, double rf, double res) :
  width(w),
  height(h),
  density(d),
  friction(f),
  rollFriction(rf),
  restitution(res)
{
        if (density < 0.0) { throw std::range_error("Negative density"); }
        if (width < 0.0)  { throw std::range_error("Negative width");  }
        if (height < 0.0)  { throw std::range_error("Negative height");  }
        if (friction < 0.0)  { throw std::range_error("Negative friction");  }
        if (rollFriction < 0.0)  { throw std::range_error("Negative roll friction");  }
        if (restitution < 0.0)  { throw std::range_error("Negative restitution");  }
        if (friction > 1.0)  { throw std::range_error("Friction > 1");  }
        if (rollFriction > 1.0)  { throw std::range_error("Roll Friction > 1");  }
        if (restitution > 1.0)  { throw std::range_error("Restitution > 1");  }
    // Postcondition
    assert(density >= 0.0);
    assert(width >= 0.0);
    assert(height >= 0.0);
    assert((friction >= 0.0) && (friction <= 1.0));
    assert((rollFriction >= 0.0) && (rollFriction <= 1.0));
    assert((restitution >= 0.0) && (restitution <= 1.0));
}

tgBox::tgBox(btRigidBody* pRigidBody, 
                const tgTags& tags,
                const double length) : 
  tgBaseRigid(pRigidBody, tags),
  m_length(length)
{
        if (pRigidBody == NULL)
    {
            throw std::invalid_argument("Pointer to btRigidBody is NULL");
    }

    // Postcondition
    assert(invariant());
    assert(m_pRigidBody == pRigidBody);
}

tgBox::~tgBox() { }

void tgBox::onVisit(const tgModelVisitor& v) const
{
    v.render(*this);
    // Do we need to render the base class?
}

void tgBox::teardown()
{

  tgBaseRigid:teardown();

  // Postcondition
  // This does not preserve the invariant
}

bool tgBox::invariant() const
{
  return
    (m_length >= 0.0);
}
