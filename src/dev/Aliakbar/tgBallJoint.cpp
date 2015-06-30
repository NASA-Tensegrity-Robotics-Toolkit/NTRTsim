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
 * @file tgBallJoint.cpp
 * @brief Contains the definition of class tgBallJoint. A Ball Joint (Socket ball).
 * @author Aliakbar Toghyan
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgBallJoint.h"

// The Bullet Physics library
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorldBulletPhysicsImpl.h"

// The C++ Standard Library
#include <cmath>
#include <stdexcept>

/**
 * Dummy constructor to make the compiler happy.
 * @todo remove this
 */
tgBallJoint::Config::Config()
{
    throw std::invalid_argument("Failed to provide arguments to tgBallJoint::Config");
}

tgBallJoint::Config::Config(
                double maxMotorImpulse,
                double maxMotorVelocity,
                double eps,
                double softness,
                double bias,
                double relaxation
        ) :
m_maxMotorImpulse(maxMotorImpulse),
m_maxMotorVelocity(maxMotorVelocity),
m_eps(eps),
m_softness(softness),
m_bias(bias),
m_relaxation(relaxation)
{
}

tgBallJoint::tgBallJoint(
        btPoint2PointConstraint* constraint,
        const tgTags& tags,
        tgBallJoint::Config& config) :
    m_balljoint(constraint),
    tgModel(tags),
    m_config(config)
{
}

tgBallJoint::~tgBallJoint()
{
    teardown();
}


void tgBallJoint::setup(tgWorld& world)
{
    assert(m_balljoint != NULL);

    //add constraint to world
    tgWorldBulletPhysicsImpl& bulletWorld =
        (tgWorldBulletPhysicsImpl&)world.implementation();
    bulletWorld.addConstraint(m_balljoint);

    // All the heavy lifting is done by info
    notifySetup();
    tgModel::setup(world);
}

void tgBallJoint::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}

void tgBallJoint::step(double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive.");
    }
    else
    { 
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}
