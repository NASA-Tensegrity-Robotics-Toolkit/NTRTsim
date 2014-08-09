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

/**
 * Dummy constructor to make the compiler happy.
 * @todo remove this
 */
tgPrismatic::Config::Config()
{
    throw std::invalid_argument("Failed to provide arguments to tgPrismatic::Config");
}

tgPrismatic::Config::Config(
                double maxLength,
                double minLength,
                double maxMotorForce,
                std::size_t segments
        ) :
m_maxLength(maxLength),
m_minLength(minLength),
m_maxMotorForce(maxMotorForce),
m_segments(segments)
{
}

tgPrismatic::tgPrismatic(
        btRigidBody* body1,
        btVector3 pos1,
        btRigidBody* body2,
        btVector3 pos2,
        const tgTags& tags,
        tgPrismatic::Config& config) :
    tgModel(tags),
    m_body1(body1),
    m_pos1(body1->getWorldTransform().inverse() * pos1),
    m_body2(body2),
    m_pos2(body2->getWorldTransform().inverse() * pos2),
    m_config(config)
{
    init();
}

tgPrismatic::tgPrismatic(
        btRigidBody* body1,
        btVector3 pos1,
        btRigidBody* body2,
        btVector3 pos2,
        std::string space_separated_tags,
        tgPrismatic::Config& config) :
    tgModel(space_separated_tags),
    m_body1(body1),
    m_pos1(body1->getWorldTransform().inverse() * pos1),
    m_body2(body2),
    m_pos2(body2->getWorldTransform().inverse() * pos2),
    m_config(config)
{
    init();
}

tgPrismatic::~tgPrismatic()
{
    if (m_slider)
    {
        dynWorld->removeConstraint(m_slider);
        delete m_slider;
    }
}

void tgPrismatic::init()
{
    btTransform transATop;
    btTransform transBTop;
    transATop.setIdentity();
    transBTop.setIdentity();
    transATop.setOrigin(m_pos1);
    transBTop.setOrigin(m_pos2);
    transATop.setRotation(btQuaternion(btVector3(0,0,1),M_PI/2));
    transBTop.setRotation(btQuaternion(btVector3(0,0,1),M_PI/2));
    m_slider = new btSliderConstraint(*m_body1, *m_body2, transATop, transBTop, true);
    m_slider->setLowerLinLimit(m_config.m_minLength);
    m_slider->setUpperLinLimit(m_config.m_maxLength);
//    m_slider->setPoweredLinMotor(true);
//    m_slider->setMaxLinMotorForce(m_config.m_maxMotorForce);
}

void tgPrismatic::setup(tgWorld& world)
{
    assert(m_slider != NULL);
//    btSoftRigidDynamicsWorld dynWorld = tgBulletUtil::worldToDynamicsWorld(world);
    dynWorld = &tgBulletUtil::worldToDynamicsWorld(world);
    dynWorld->addConstraint(m_slider);
    // All the heavy lifting is done by info
    notifySetup();
    tgModel::setup(world);
}

void tgPrismatic::teardown()
{
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
        tgModel::step(dt);  // Step any children
    }
}

void tgPrismatic::moveMotors(double dt)
{
}
