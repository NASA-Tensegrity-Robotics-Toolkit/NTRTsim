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
 * @file tgWorld.cpp
 * @brief Contains the definitions of members of class tgWorld
 * $Id$
 */

// This module
#include "tgWorld.h"
// This application
#include "tgWorldBulletPhysicsImpl.h"
#include "terrain/tgBoxGround.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>

tgWorld::Config::Config(double g, double ws) :
gravity(g),
worldSize(ws)
{
  if (ws <= 0.0)
  {
    throw std::invalid_argument("worldSize is not postive");
  }
}

/**
 * @todo Use the factory method design pattern to
 * create the m_pImpl object.
 */
tgWorld::tgWorld() :
  m_config(),
  m_pGround(new tgBoxGround()),
  m_pImpl(new tgWorldBulletPhysicsImpl(m_config, (tgBulletGround*)m_pGround))
{
  // Postcondition
  assert(invariant());
}

/**
 * @todo Use the factory method design pattern to
 * create the m_pImpl object.
 */
tgWorld::tgWorld(const tgWorld::Config& config) :
  m_config(config),
  m_pGround(new tgBoxGround()),
  m_pImpl(new tgWorldBulletPhysicsImpl(m_config, (tgBulletGround*)m_pGround))
{
  // Postcondition
  assert(invariant());
}

/**
 * @todo Use the factory method design pattern to
 * create the m_pImpl object.
 */
tgWorld::tgWorld(const tgWorld::Config& config, tgGround* ground) :
  m_config(config),
  m_pGround(ground),
  m_pImpl(new tgWorldBulletPhysicsImpl(m_config, (tgBulletGround*)m_pGround))
{
  // Postcondition
  assert(invariant());
}

tgWorld::~tgWorld()
{
  delete m_pImpl;
  delete m_pGround;
}

void tgWorld::reset()
{
  delete m_pImpl;
  m_pImpl = new tgWorldBulletPhysicsImpl(m_config, (tgBulletGround*)m_pGround);
  // Postcondition
  assert(invariant());
}

void tgWorld::reset(const tgWorld::Config& config)
{
  // Update the config
  m_config = config;
  // Reset as usual
  reset();

  // Postcondition
  assert(invariant());
}

void tgWorld::reset(tgGround * ground)
{
    delete m_pGround;
    
    m_pGround = ground;
    
    // Reset as usual
    reset();
}

void tgWorld::step(double dt) const
{
  if (dt <= 0.0)
  {
    throw std::invalid_argument("dt is not postive");
  }
  else
  {
    // Forward to the implementation
    m_pImpl->step(dt);
  }
}

bool tgWorld::invariant() const
{
  return m_pImpl != 0;
}
