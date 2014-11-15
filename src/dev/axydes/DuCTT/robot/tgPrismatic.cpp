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
 * @file tgPrismatic.cpp
 * @brief Contains the definition of class tgPrismatic. A prismatic actuator.
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgPrismatic.h"

#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorldBulletPhysicsImpl.h"

// The C++ Standard Library
#include <cmath>
#include <stdexcept>

/**
 * Dummy constructor to make the compiler happy.
 * @todo remove this
 */
tgPrismatic::Config::Config()
{
    throw std::invalid_argument("Failed to provide arguments to tgPrismatic::Config");
}

tgPrismatic::Config::Config(
                double axis,
                double rotation,
                double minLength,
                double maxLength,
                double maxMotorForce,
                double maxVelocity,
                double eps
        ) :
m_axis(axis),
m_rotation(rotation),
m_minLength(minLength),
m_maxLength(maxLength),
m_maxMotorForce(maxMotorForce),
m_maxVelocity(maxVelocity),
m_eps(eps)
{
}

tgPrismatic::tgPrismatic(
        btSliderConstraint* constraint,
        const tgTags& tags,
        tgPrismatic::Config& config) :
    tgModel(tags),
    m_slider(constraint),
    m_config(config)
{
    init();
}


tgPrismatic::~tgPrismatic()
{
    teardown();
}

void tgPrismatic::init()
{
    m_slider->setLowerLinLimit(m_config.m_minLength);
    m_slider->setUpperLinLimit(m_config.m_maxLength);
    m_slider->setPoweredLinMotor(true);
    m_slider->setMaxLinMotorForce(m_config.m_maxMotorForce);
    m_preferredLength = m_config.m_minLength;
}

void tgPrismatic::setup(tgWorld& world)
{
    assert(m_slider != NULL);

    //add constraint to world
    tgWorldBulletPhysicsImpl& bulletWorld =
      (tgWorldBulletPhysicsImpl&)world.implementation();
    bulletWorld.addConstraint(m_slider);

    // All the heavy lifting is done by info
    notifySetup();
    tgModel::setup(world);
}

void tgPrismatic::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}

void tgPrismatic::step(double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive.");
    }
    else
    { 
        notifyStep(dt);
        moveMotors(dt);
        tgModel::step(dt);  // Step any children
    }
}

bool tgPrismatic::setPreferredLength(double length)
{
    bool success = true;
    if (length > m_config.m_maxLength)
    {
        m_preferredLength = m_config.m_maxLength;
        success = false;
    }
    else if (length < m_config.m_minLength)
    {
        m_preferredLength = m_config.m_minLength;
        success = false;
    }
    else
    {
        m_preferredLength = length;
    }

    return success;
}

void tgPrismatic::setMaxVelocity(double vel)
{
    m_config.m_maxVelocity = vel;
}

void tgPrismatic::setMaxForce(double force)
{
    m_config.m_maxMotorForce = force;
}

void tgPrismatic::moveMotors(double dt)
{
    double linDepth = getActualLength();

    double targetVel = m_preferredLength - linDepth;
    if (targetVel > m_config.m_maxVelocity)
        targetVel = m_config.m_maxVelocity;
    if (targetVel < -m_config.m_maxVelocity)
        targetVel = -m_config.m_maxVelocity;

    if (isAtPreferredLength())
    {
        m_slider->setLowerLinLimit(linDepth);
        m_slider->setUpperLinLimit(linDepth);
        m_slider->setTargetLinMotorVelocity(0);
    }
    else if (linDepth < m_preferredLength)
    {
        m_slider->setLowerLinLimit(m_config.m_minLength);
        m_slider->setUpperLinLimit(m_config.m_maxLength);
        m_slider->setTargetLinMotorVelocity(targetVel/dt);
    }
    else if (linDepth > m_preferredLength)
    {
        m_slider->setLowerLinLimit(m_config.m_minLength);
        m_slider->setUpperLinLimit(m_config.m_maxLength);
        m_slider->setTargetLinMotorVelocity(targetVel/dt);
    }
    else
    {
        m_slider->setLowerLinLimit(linDepth);
        m_slider->setUpperLinLimit(linDepth);
        m_slider->setTargetLinMotorVelocity(0);
    }
}

double tgPrismatic::getActualLength()
{
    return m_slider->getLinearPos();
}

bool tgPrismatic::isAtPreferredLength()
{
    return fabs(getActualLength() - m_preferredLength) <= m_config.m_eps;
}
