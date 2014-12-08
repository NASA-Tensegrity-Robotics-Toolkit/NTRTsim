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
 * @file tgImpedanceController.cpp
 * @brief Contains the definition of members of class tgImpedanceController.
 * $Id$
 */

// This module
#include "tgImpedanceController.h"
// This library
#include "tgBasicController.h"
#include "core/tgBaseString.h"
#include "core/tgCast.h"
/// @todo create and use new string base class
#include "tgKinematicString.h"

/**
 * The default value for Controller::tgImpedanceController::m_offsetTension
 * @author Lee Brownston
 * @date Fri 31 Jan 2014
 */
static const double kDefaultOffsetTension = 0.001;

/**
 * The default value for Controller::tgImpedanceController::m_offsetTension
 * @author Lee Brownston
 * @date Fri 31 Jan 2014
 */
static const double kDefaultLengthStiffness = 0.0;

/**
 * The default value for Controller::tgImpedanceController::m_offsetTension
 * @author Lee Brownston
 * @date Fri 31 Jan 2014
 */
static const double kDefaultVelocityStiffness = 0.0;

tgImpedanceController::tgImpedanceController() :
  m_offsetTension(kDefaultOffsetTension),
  m_lengthStiffness(kDefaultLengthStiffness),
  m_velStiffness(kDefaultVelocityStiffness)
{
        // Postcondition
        assert(invariant());
    assert(m_offsetTension == kDefaultOffsetTension);
    assert(m_lengthStiffness == kDefaultLengthStiffness);
    assert(m_velStiffness == kDefaultVelocityStiffness);
}

tgImpedanceController::tgImpedanceController(double offsetTension,
                           double lengthStiffness,
                           double velStiffness) :
  m_offsetTension(offsetTension),
  m_lengthStiffness(lengthStiffness),
  m_velStiffness(velStiffness)
{
        // Precondition
        assert(offsetTension >= 0.0);
    assert(lengthStiffness >= 0.0);
    assert(velStiffness >= 0.0);

    // Postcondition
        assert(invariant());
    assert(m_offsetTension == offsetTension);
    assert(m_lengthStiffness == lengthStiffness);
    assert(m_velStiffness == velStiffness);
}

/**
 * Calculate and return the set force from three components: offset,
 * displacement and velocity.
 * If the result is negative, return 0.
 * @param[in] offset the offset component of force
 * @param[in] displacement the displacement component of force
 * @param[in] velocity the velocity component of force
 * @retval 0.0 if offset + displacement + velocity is negative
 * @return offset + displacement + velocity if this sum is not negative
 * @author Lee Brownston
 * @date Thu 30 Jan 2014
 */
static inline double determineSetTension(double offset,
                       double displacement,
                       double velocity)
{
  return std::max(static_cast<double>(0.0), offset + displacement + velocity);
}

double
tgImpedanceController::control(tgBasicController& mLocalController, 
                                 double deltaTimeSeconds,
                                 double newPosition,
                                 double offsetVel)
{
    return controlTension(  mLocalController,
                            deltaTimeSeconds,
                            newPosition,
                            m_offsetTension,
                            offsetVel);
}

double
tgImpedanceController::controlTension(tgBasicController& mLocalController,
                                 double deltaTimeSeconds,
                                 double newPosition ,
                                 double offsetTension,
                                 double offsetVel)
{
	
	/// @todo create and use new string base class
	const tgKinematicString* mString = tgCast::cast<tgControllable, tgKinematicString>
											(mLocalController.getControllable());
	
	assert (mString);
	
    const double actualLength = mString->getCurrentLength();
    const double vel = mString->getVelocity();

    const double setTension = 
      determineSetTension(offsetTension,
                m_lengthStiffness * (actualLength - newPosition),
                m_velStiffness * (vel - offsetVel));
	
	const double currentTension = mString->getTension();
	
    mLocalController.control(deltaTimeSeconds, setTension, currentTension);

    // Postcondition
    assert(setTension >= 0.0);

    return setTension;
}

void tgImpedanceController::setOffsetTension(double offsetTension)
{
        // Precondition
        assert(offsetTension >= 0.0);

        m_offsetTension = offsetTension;

    // Postcondition
    assert(invariant());
    assert(m_offsetTension == offsetTension);
}

void tgImpedanceController::setLengthStiffness(double lengthStiffness)
{
        // Precondition
        assert(lengthStiffness >= 0.0);

        m_lengthStiffness = lengthStiffness;

    // Postcondition
    assert(invariant());
    assert(m_lengthStiffness == lengthStiffness);
}

void tgImpedanceController::setVelStiffness(double velStiffness)
{
        // Precondition
        assert(velStiffness >= 0.0);

        m_velStiffness = velStiffness;

    // Postcondition
    assert(invariant());
    assert(m_velStiffness == velStiffness);
}

bool tgImpedanceController::invariant() const
{
        return
      (m_offsetTension >= 0.0)   &&
      (m_lengthStiffness >= 0.0) &&
      (m_velStiffness >= 0.0);
}
