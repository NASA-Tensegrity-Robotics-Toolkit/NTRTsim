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
 * @file tgSpringCableActuator.cpp
 * @brief Contains the definitions of members of class tgSpringCableActuator
 * @author Brian Tietz
 * $Id$
 */

// This Module
#include "tgSpringCableActuator.h"
#include "tgSpringCable.h"
#include "tgWorld.h"
// The C++ Standard Library
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace std;

tgSpringCableActuator::Config::Config(double s,
                   double d,
                   double p,
                   bool h,
                   double mf,
                   double tVel,
                   double mnAL,
                   double mnRL,
		   double rot,
   	           bool moveCPA,
		   bool moveCPB) :
  stiffness(s),
  damping(d),
  pretension(p),
  hist(h),
  maxTens(mf),
  targetVelocity(tVel),
  minActualLength(mnAL),
  minRestLength(mnRL),
  rotation(rot),
  moveCablePointAToEdge(moveCPA),
  moveCablePointBToEdge(moveCPB)
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
    /* Pretension is checked in Muscle2P, and can be any value
     * i.e. starting with a slack string
     */
    else if (mf < 0.0)
    {
        throw std::invalid_argument("max tension is negative.");
    }
    else if (mnAL < 0.0)
    {
        throw std::invalid_argument("min Actual Length is negative.");
    }
    else if (mnRL < 0.0)
    {
        throw std::invalid_argument("min Rest Length is negative.");
    }
    else if (abs(rot) > M_PI * 2.0)
    {
         throw std::invalid_argument("Abs of rotation is greater than 2pi. Are you sure you're setting the right parameters?");
    }
}

void tgSpringCableActuator::Config::scale (double sf)
{
  pretension	  *= sf;
  maxTens         *= sf;
  targetVelocity  *= sf;
  minActualLength *= sf;
  minRestLength   *= sf;
}



void tgSpringCableActuator::constructorAux()
{
    if (m_config.targetVelocity < 0.0)
    {
        throw std::invalid_argument("Target velocity is negative.");
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
tgSpringCableActuator::tgSpringCableActuator(tgSpringCable* springCable,
                    const tgTags& tags,
                   tgSpringCableActuator::Config& config) :
    tgModel(tags),
    m_springCable(springCable),
    m_config(config),
    m_pHistory(new SpringCableActuatorHistory()),
    m_restLength(springCable->getRestLength()),
    m_startLength(springCable->getActualLength()),
    m_prevVelocity(0.0)
{
    constructorAux();

    // Postcondition
    assert(invariant());
    assert(m_springCable == springCable);
}

tgSpringCableActuator::~tgSpringCableActuator()
{
    delete m_springCable;
    delete m_pHistory;
}
    
void tgSpringCableActuator::setup(tgWorld& world)
{
    tgModel::setup(world);
}

void tgSpringCableActuator::teardown()
{
    tgModel::teardown();
}
    
void tgSpringCableActuator::step(double dt) 
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

const double tgSpringCableActuator::getStartLength() const
{
    return m_startLength;
}
    
const double tgSpringCableActuator::getCurrentLength() const
{
    return m_springCable->getActualLength();
}  

const double tgSpringCableActuator::getTension() const
{
    return m_springCable->getTension();
}
    
const double tgSpringCableActuator::getRestLength() const
{
    return m_springCable->getRestLength();
}

const double tgSpringCableActuator::getVelocity() const
{
    return m_springCable->getVelocity();
}

const tgSpringCableActuator::SpringCableActuatorHistory& tgSpringCableActuator::getHistory() const
{
    return *m_pHistory;
}

bool tgSpringCableActuator::invariant() const
{
    return
      // Only know this about history here, can't confirm how child
      // classes need to log
      (m_pHistory != NULL) && 
      (m_config.targetVelocity >= 0.0) &&
      (m_config.minActualLength >= 0.0) &&
      (m_startLength >= 0.0);
}
