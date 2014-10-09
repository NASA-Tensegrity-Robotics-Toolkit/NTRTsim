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
 * @file tgHinge.cpp
 * @brief Contains the definition of class tgHinge. A hinge actuator.
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgHinge.h"

#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorldBulletPhysicsImpl.h"

// The C++ Standard Library
#include <cmath>
#include <stdexcept>

/**
 * Dummy constructor to make the compiler happy.
 * @todo remove this
 */
tgHinge::Config::Config()
{
    throw std::invalid_argument("Failed to provide arguments to tgHinge::Config");
}

tgHinge::Config::Config(
                double maximum,
                double minimum,
                int axisFrom,
                int axisTo,
                bool useMotor,
                double maxMotorImpulse,
                double maxMotorVelocity,
                double eps
        ) :
m_maximum(maximum),
m_minimum(minimum),
m_axisFrom(axisFrom),
m_axisTo(axisTo),
m_useMotor(useMotor),
m_maxMotorImpulse(maxMotorImpulse),
m_maxMotorVelocity(maxMotorVelocity),
m_eps(eps)
{
}

tgHinge::tgHinge(
        btHingeConstraint* constraint,
        const tgTags& tags,
        tgHinge::Config& config) :
    tgModel(tags),
    m_hinge(constraint),
    m_config(config)
{
    init();
}

tgHinge::~tgHinge()
{
    teardown();
}

void tgHinge::init()
{
    m_hinge->setLimit(m_config.m_minimum, m_config.m_maximum);
    m_hinge->setMaxMotorImpulse(m_config.m_maxMotorImpulse);
    m_preferredAngle = m_config.m_minimum;
}

void tgHinge::setup(tgWorld& world)
{
    assert(m_hinge != NULL);

    //add constraint to world
    tgWorldBulletPhysicsImpl& bulletWorld =
      (tgWorldBulletPhysicsImpl&)world.implementation();
    bulletWorld.addConstraint(m_hinge);

    // All the heavy lifting is done by info
    notifySetup();
    tgModel::setup(world);
}

void tgHinge::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}

void tgHinge::step(double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive.");
    }
    else
    { 
        notifyStep(dt);
        if (m_config.m_useMotor)
        {
            moveMotors(dt);
        }
        tgModel::step(dt);  // Step any children
    }
}

bool tgHinge::setPreferredAngle(double angle)
{
    bool success = true;
    if (angle > m_config.m_maximum)
    {
        m_preferredAngle = m_config.m_maximum;
        success = false;
    }
    else if (angle < m_config.m_minimum)
    {
        m_preferredAngle = m_config.m_minimum;
        success = false;
    }
    else
    {
        m_preferredAngle = angle;
    }

    return success;
}

void tgHinge::setMaxVelocity(double vel)
{
    m_config.m_maxMotorVelocity = vel;
}

void tgHinge::setMaxImpulse(double impulse)
{
    m_config.m_maxMotorImpulse = impulse;
}

void tgHinge::moveMotors(double dt)
{
    double jntAngle = 0;

    if (fabs(jntAngle - m_preferredAngle) <= m_config.m_eps)
    {
        m_hinge->setLimit(jntAngle, jntAngle);
        m_hinge->enableAngularMotor(true, 0, 0);
    }
    else if (jntAngle < m_preferredAngle)
    {
        m_hinge->setLimit(m_config.m_minimum, m_config.m_maximum);
        m_hinge->enableAngularMotor(true, m_config.m_maxMotorVelocity/dt, m_config.m_maxMotorImpulse);
    }
    else if (jntAngle > m_preferredAngle)
    {
        m_hinge->setLimit(m_config.m_minimum, m_config.m_maximum);
        m_hinge->enableAngularMotor(true, -m_config.m_maxMotorVelocity/dt, m_config.m_maxMotorImpulse);
    }
    else
    {
        m_hinge->setLimit(jntAngle, jntAngle);
        m_hinge->enableAngularMotor(true, 0, 0);
    }
}
