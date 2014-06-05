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
 * @file tgBaseString.cpp
 * @brief Contains the definitions of members of class tgBaseString
 * @author Brian Tietz
 * $Id$
 */

// This Module
#include "tgBaseString.h"
#include "tgWorld.h"
// The C++ Standard Library
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace std;

tgBaseString::Config::Config(double s,
                   double d,
                   bool h,
                   double rot,
                   double mf,
                   double tVel,
                   double mxAcc,
                   double mnAL,
                   double mnRL) :
  stiffness(s),
  damping(d),
  hist(h),
  rotation(rot),
  maxTens(mf),
  targetVelocity(tVel),
  maxAcc(mxAcc),
  minActualLength(mnAL),
  minRestLength(mnRL)         
{
    ///@todo is this the right place for this, or the constructor of this class?
    if (s < 0.0)
    {
        throw std::invalid_argument("stiffness is negative.");
    }
    else if (d < 0.0)
    {
        throw std::invalid_argument("damping is negative.");
    }
    else if (mf < 0.0)
    {
        throw std::invalid_argument("max tension is negative.");
    }
    else if (maxAcc < 0.0)
    {
        throw std::invalid_argument("max acceleration is negative.");
    }
    else if (mnAL < 0.0)
    {
        throw std::invalid_argument("min Actual Length is negative.");
    }
    else if (mnRL < 0.0)
    {
        throw std::invalid_argument("min Rest Length is negative.");
    }
}

void tgBaseString::Config::scale (double sf)
{
  maxTens         *= sf;
  targetVelocity  *= sf;
  maxAcc          *= sf;
  minActualLength *= sf;
  minRestLength   *= sf;
}



void tgBaseString::constructorAux()
{
    if (m_config.targetVelocity < 0.0)
    {
        throw std::invalid_argument("Target velocity is negative.");
    }
    else if (m_config.maxAcc < 0.0)
    {
        throw std::invalid_argument("Maximum acceleration is negative.");
    }
    else if (m_config.minActualLength < 0.0)
    {
        throw std::invalid_argument("Minimum length is negative.");
    }
    else if (m_restLength < 0.0)
    {
        throw std::invalid_argument("Starting rest length is negative.");
    }
}
tgBaseString::tgBaseString(const tgTags& tags,
                   tgBaseString::Config& config,
                   double restLength,
                   double actualLength) :
    tgModel(tags),
    m_pHistory(new BaseStringHistory()),
    m_config(config),
    m_restLength(restLength),
    m_preferredLength(m_restLength),
    m_startLength(actualLength),
    m_prevVelocity(0.0)
{
    constructorAux();

    // Postcondition
    assert(invariant());
    assert(m_preferredLength == m_restLength);
}

tgBaseString::tgBaseString(std::string space_separated_tags,
                   tgBaseString::Config& config,
                   double restLength,
                   double actualLength) :
    tgModel(space_separated_tags),
    m_pHistory(new BaseStringHistory()),
    m_config(config),
    m_restLength(restLength),
    m_preferredLength(m_restLength),
    m_startLength(actualLength),
    m_prevVelocity(0.0)
{
    constructorAux();

    // Postcondition
    assert(invariant());
    assert(m_preferredLength == m_restLength);
}

tgBaseString::~tgBaseString()
{
    delete m_pHistory;
}
    
void tgBaseString::setup(tgWorld& world)
{
    tgModel::setup(world);
}

void tgBaseString::teardown()
{
    tgModel::teardown();
}
    
void tgBaseString::step(double dt) 
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive.");
    }
    else
    {   
        tgModel::step(dt);
    }
}
 
void tgBaseString::setRestLength(double newLength, float dt)
{
    if (newLength < 0.0)
    {
      throw std::invalid_argument("Rest length is negative.");
    }
    else
    {
        m_preferredLength = newLength;      
        moveMotors(dt);
    }

    // Postcondition
    assert(invariant());
    assert(m_preferredLength == newLength);
}

bool tgBaseString::invariant() const
{
    return
      // Only know this about history here, can't confirm how child
      // classes need to log
      (m_pHistory != NULL) && 
      (m_config.targetVelocity >= 0.0) &&
      (m_config.maxAcc >= 0.0) &&
      (m_config.minActualLength >= 0.0) &&
      (m_preferredLength >= 0.0) &&
      (m_startLength >= 0.0);
}
