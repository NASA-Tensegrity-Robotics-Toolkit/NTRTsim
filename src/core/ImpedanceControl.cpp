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
 * @file ImpedanceControl.cpp
 * @brief Contains the definition of members of class ImpedanceControl.
 * $Id$
 */

// This module
#include "ImpedanceControl.h"
// This library
#include "tgBaseString.h"

/**
 * The default value for Controller::ImpedanceControl::_offsetTension
 * @author Lee Brownston
 * @date Fri 31 Jan 2014
 */
static const double kDefaultOffsetTension = 0.001;

/**
 * The default value for Controller::ImpedanceControl::_offsetTension
 * @author Lee Brownston
 * @date Fri 31 Jan 2014
 */
static const double kDefaultLengthStiffness = 0.0;

/**
 * The default value for Controller::ImpedanceControl::_offsetTension
 * @author Lee Brownston
 * @date Fri 31 Jan 2014
 */
static const double kDefaultVelocityStiffness = 0.0;

ImpedanceControl::ImpedanceControl() :
  _offsetTension(kDefaultOffsetTension),
  _lengthStiffness(kDefaultLengthStiffness),
  _velStiffness(kDefaultVelocityStiffness)
{
        // Postcondition
        assert(invariant());
    assert(_offsetTension == kDefaultOffsetTension);
    assert(_lengthStiffness == kDefaultLengthStiffness);
    assert(_velStiffness == kDefaultVelocityStiffness);
}

ImpedanceControl::ImpedanceControl(double offsetTension,
                           double lengthStiffness,
                           double velStiffness) :
  _offsetTension(offsetTension),
  _lengthStiffness(lengthStiffness),
  _velStiffness(velStiffness)
{
        // Precondition
        assert(offsetTension >= 0.0);
    assert(lengthStiffness >= 0.0);
    assert(velStiffness >= 0.0);

    // Postcondition
        assert(invariant());
    assert(_offsetTension == offsetTension);
    assert(_lengthStiffness == lengthStiffness);
    assert(_velStiffness == velStiffness);
}

/**
 * Calculate and return the velocity from the change of position and the change
 * of time.
 * If time is not positive, return 0.
 * @param[in] ds the change of position
 * @param[in] dt the change of time
 * @retval ds/dt if dt > 0.0
 * @retval 0.0 if dt <= 0
 * @author Lee Brownston
 * @date Thu 30 Jan 2014
 */
static inline double determineVelocity(double ds, double dt)
{
        return (dt > 0.0) ? (ds / dt) : 0.0;
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
ImpedanceControl::control(tgBaseString* const mString, 
                                 double deltaTimeSeconds,
                                 double newPosition,
                                 double offsetVel)
{
    return controlTension(  mString,
                            deltaTimeSeconds,
                            newPosition,
                            _offsetTension,
                            offsetVel);
}

double
ImpedanceControl::controlTension(tgBaseString* const mString, 
                                 double deltaTimeSeconds,
                                 double newPosition ,
                                 double offsetTension,
                                 double offsetVel)
{
    // Precondition
    assert(mString != NULL);

    const double actualLength = mString->getRestLength();
    const double vel = mString->getVelocity();

    const double setTension = 
      determineSetTension(offsetTension,
                _lengthStiffness * (actualLength - newPosition),
                _velStiffness * (vel - offsetVel));

    mString->tensionMinLengthController(setTension,
                        deltaTimeSeconds);

    // Postcondition
    assert(setTension >= 0.0);

    return setTension;
}

void ImpedanceControl::setOffsetTension(double offsetTension)
{
        // Precondition
        assert(offsetTension >= 0.0);

        _offsetTension = offsetTension;

    // Postcondition
    assert(invariant());
    assert(_offsetTension == offsetTension);
}

void ImpedanceControl::setLengthStiffness(double lengthStiffness)
{
        // Precondition
        assert(lengthStiffness >= 0.0);

        _lengthStiffness = lengthStiffness;

    // Postcondition
    assert(invariant());
    assert(_lengthStiffness == lengthStiffness);
}

void ImpedanceControl::setVelStiffness(double velStiffness)
{
        // Precondition
        assert(velStiffness >= 0.0);

        _velStiffness = velStiffness;

    // Postcondition
    assert(invariant());
    assert(_velStiffness == velStiffness);
}

bool ImpedanceControl::invariant() const
{
        return
      (_offsetTension >= 0.0)   &&
      (_lengthStiffness >= 0.0) &&
      (_velStiffness >= 0.0);
}
