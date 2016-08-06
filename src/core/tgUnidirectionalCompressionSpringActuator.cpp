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
 * @file tgUnidirectionalCompressionSpringActuator.cpp
 * @brief Contains the definitions of members of class 
 * tgUnidirectionalCompressionSpringActuator.
 * @author Drew Sabelhaus, Brian Mirletz
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// This Module
#include "tgUnidirectionalCompressionSpringActuator.h"
// NTRT core library files
#include "core/tgBulletUnidirectionalCompressionSpring.h"
#include "core/tgModelVisitor.h"
#include "core/tgWorld.h"
// The Bullet Physics Library
#include "LinearMath/btQuickprof.h"

// The C++ Standard Library
#include <cmath>
#include <deque> // For history
#include <iostream>
#include <stdexcept>

using namespace std;

// Declaration of the config struct, now containing the additional parameter
// for direction
// Note that this Config struct includes the config struct for its superclass.
tgUnidirectionalCompressionSpringActuator::Config::Config(bool iFEA,
		   double s,
                   double d,
		   double rL,
		   btVector3 dir) :
  tgCompressionSpringActuator::Config::Config(iFEA, s, d, rL),
  direction(dir)       
{
    ///@todo is this the right place for this, or the constructor of this class?
  /*
   * TO-DO: perform a check on dir.
  if ()
    {
        throw std::invalid_argument("");
    }
  */
}

// A helper function for the constructor.
// @TO-DO: Where is the "correct" place for these non-negative checks?
void tgUnidirectionalCompressionSpringActuator::constructorAux()
{
    if (m_compressionSpring == NULL)
    {
        throw std::invalid_argument("Pointer to tgBulletCompressionSpring is NULL.");
    }
    /* Some check about direction
    else if (m_config.restLength < 0.0)
    {
        throw std::invalid_argument("Rest length is less than zero..");
    }*/
}

/**
 * The main constructor for this class.
 * Note that since direction is part of config, there is no private
 * variable for it that needs to be assigned in this constructor.
 * @TO-DO: confirm the polymorphism of tgCompressionSpringActuator taking
 * a tgBulletUnidirectionalCompressionSpring, 
 * since that's a subclass of tgBulletCompressionSpring...
 */
tgUnidirectionalCompressionSpringActuator::tgUnidirectionalCompressionSpringActuator(
		   tgBulletUnidirectionalCompressionSpring* compressionSpring,
                   const tgTags& tags,
                   tgUnidirectionalCompressionSpringActuator::Config& config) :
    m_config(config),
    tgCompressionSpringActuator(compressionSpring, tags, config)
{
    // call the helper function that does some checks
    constructorAux();

    // Postcondition
    assert(invariant());
    // making sure that no changes to the aux constructor function end up
    // changing around the compression spring pointer.
    assert(m_compressionSpring == compressionSpring);
}

tgUnidirectionalCompressionSpringActuator::~tgUnidirectionalCompressionSpringActuator()
{
    std::cout << "deleting compression spring within tgCompressionSpringActuator" << std::endl;
    delete m_compressionSpring;
    // This may not need to happen. Does ~tgCompressonSpringActuator get called
    // automatically? ...
}
    
void tgUnidirectionalCompressionSpringActuator::setup(tgWorld& world)
{
    // This needs to be called here in case the controller needs to cast
    notifySetup();
    tgModel::setup(world);
}

void tgUnidirectionalCompressionSpringActuator::teardown()
{
    // Do not notify teardown. Copied from tgBasicActuator/tgSpringCableActuator.
    tgModel::teardown();
}

/**
 * The step function calls on the tgUnidirectionalBulletCompressionSpring 
 * to apply forces.
 * Since this class does not currently do any actuation, this function is the
 * only place that things get updated at each timestep.
 */
void tgUnidirectionalCompressionSpringActuator::step(double dt) 
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgUnidirectionalCompressionSpringActuator::step");
#endif //BT_NO_PROFILE   	
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive.");
    }
    else
    {   
        // Want to update any controls before applying forces
        notifyStep(dt);
	// This should call the step function within
	// tgBulletUnidirectionalCompressionSpring instead of the one inside
	// tgBulletCompressionSpring.
        m_compressionSpring->step(dt);
        tgModel::step(dt);
    }
}

// Renders the spring in the NTRT window
void tgUnidirectionalCompressionSpringActuator::onVisit(const tgModelVisitor& r) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgUnidirectionalCompressionSpringActuator::onVisit");
#endif //BT_NO_PROFILE	
    r.render(*this);
}

// Finally, the invariant for assertions.
bool tgUnidirectionalCompressionSpringActuator::invariant() const
{
    return
      (m_compressionSpring != NULL);
}
