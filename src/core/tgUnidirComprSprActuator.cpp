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
 * @file tgUnidirComprSprActuator.cpp
 * @brief Contains the definitions of members of class 
 * tgUnidirComprSpringAct.
 * @author Drew Sabelhaus
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// This Module
#include "tgUnidirComprSprActuator.h"
// NTRT core library files
#include "core/tgBulletUnidirComprSpr.h"
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

// The constructor for this class' Config struct,
// now containing the additional parameter for direction.
// (This is NOT a declaration of the config struct, that's in the .h file.
//  Remember that in C++, structs have constructor methods.)
// Note that this Config struct includes the config struct for its superclass.
tgUnidirComprSprActuator::Config::Config(bool iFEA,
		   double s,
                   double d,
		   double rL,
		   bool moveCPA,
		   bool moveCPB,
		   btVector3 * dir) :
  tgCompressionSpringActuator::Config::Config(iFEA, s, d, rL, moveCPA, moveCPB),
  direction(dir)       
{
    // Direction cannot be null.
    if( dir == NULL)
    {
      throw std::invalid_argument("Pointer to btVector3 direction is NULL, inside constructor for tgUnidirComprSprActuator Config.");
    }
  
    // Debugging
    #if (0)
    std::cout << "tgUnidirComprSprActuator Config constructor. Direction is:" << std::endl;
    std::cout << "(" << direction->x() << ",";
    std::cout << direction->y() << ",";
    std::cout << direction->z() << ")" << std::endl;
    #endif  
}

// A helper function for the constructor.
// @TO-DO: Where is the "correct" place for these non-negative checks?
void tgUnidirComprSprActuator::constructorAux()
{
    if (m_compressionSpring == NULL)
    {
        throw std::invalid_argument("Pointer to tgBulletUnidirComprSpr is NULL.");
    }

    // Direction cannot be null.
    if( m_config.direction == NULL)
    {
      throw std::invalid_argument("Pointer to btVector3 direction is NULL, inside constructor aux for tgUnidirComprSprActuator.");
    }

    // Debugging
    #if (0)
    std::cout << "tgUnidirComprSprActuator constructor aux. Direction is:" << std::endl;
    std::cout << "(" << m_config.direction->x() << ",";
    std::cout << m_config.direction->y() << ",";
    std::cout << m_config.direction->z() << ")" << std::endl;
    #endif  
}

/**
 * The main constructor for this class.
 * Note that since direction is part of config, there is no private
 * variable for it that needs to be assigned in this constructor.
 * @TO-DO: confirm the polymorphism of tgCompressionSpringActuator taking
 * a tgBulletUnidirComprSpr, 
 * since that's a subclass of tgBulletCompressionSpring...
 */
tgUnidirComprSprActuator::tgUnidirComprSprActuator(
		   tgBulletUnidirComprSpr* compressionSpring,
                   const tgTags& tags,
                   tgUnidirComprSprActuator::Config& config) :
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

tgUnidirComprSprActuator::~tgUnidirComprSprActuator()
{
    //Debugging
    #if(0)
    std::cout << "Destroying tgUnidirComprSprActuator" << std::endl;
    std::cout << "This class does not delete m_compressionSpring." << std::endl;
    #endif

}
    
void tgUnidirComprSprActuator::setup(tgWorld& world)
{
    // This needs to be called here in case the controller needs to cast
    notifySetup();
    tgModel::setup(world);
}

void tgUnidirComprSprActuator::teardown()
{
    // Copied from tgBasicActuator/tgSpringCableActuator.
    tgModel::teardown();
}

/**
 * The step function calls on the tgBulletUnidirComprSpr to apply forces.
 * Since this class does not currently do any actuation, this function is the
 * only place that things get updated at each timestep.
 */
void tgUnidirComprSprActuator::step(double dt) 
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgUnidirComprSprActuator::step");
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
	// tgBulletUnidirComprSpr instead of the one inside
	// tgBulletCompressionSpring.
        m_compressionSpring->step(dt);
        tgModel::step(dt);
    }
}

// Renders the spring in the NTRT window
void tgUnidirComprSprActuator::onVisit(const tgModelVisitor& r) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgUnidirComprSprActuator::onVisit");
#endif //BT_NO_PROFILE	
    r.render(*this);
}

// Finally, the invariant for assertions.
bool tgUnidirComprSprActuator::invariant() const
{
    return
      (m_compressionSpring != NULL &&
      m_config.direction != NULL);
}
