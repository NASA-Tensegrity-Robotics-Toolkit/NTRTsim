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
 * @file tgPrismaticInfo.cpp
 * @brief Contains the definition of members of the class tgPrismatic. A prismatic actuator builder.
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgPrismaticInfo.h"

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"

#include <iostream>

#include "tgcreator/tgNode.h"
#include "tgcreator/tgStructureInfo.h"

tgPrismaticInfo::tgPrismaticInfo(const tgPrismatic::Config& config) :
    m_config(config),
    tgConnectorInfo() 
{
}

tgPrismaticInfo::tgPrismaticInfo(const tgPrismatic::Config& config, tgTags tags) :
    m_config(config),
    tgConnectorInfo(tags)
{}

tgPrismaticInfo::tgPrismaticInfo(const tgPrismatic::Config& config, const tgPair& pair) :
    m_config(config),
    tgConnectorInfo(pair)
{}

tgPrismaticInfo::~tgPrismaticInfo()
{
}

tgConnectorInfo* tgPrismaticInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgPrismaticInfo(m_config, pair);
}

void tgPrismaticInfo::initConnector(tgWorld& world)
{
}

btSliderConstraint* tgPrismaticInfo::createSlider()
{
    btRigidBody* fromBody = getFromRigidBody();
    btVector3 from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo(), 0);//m_config.rotation);
    btTransform transATop;
    transATop.setIdentity();
    transATop.setOrigin(fromBody->getWorldTransform().inverse() * from);
    transATop.setRotation(btQuaternion(btVector3(0,0,1),M_PI/2));

    btRigidBody* toBody = getToRigidBody();
    btVector3 to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom(), 0);//m_config.rotation);
    btTransform transBTop;
    transBTop.setIdentity();
    transBTop.setOrigin(toBody->getWorldTransform().inverse() * to);
    transBTop.setRotation(btQuaternion(btVector3(0,0,1),M_PI/2));

    btSliderConstraint* slider = new btSliderConstraint(*fromBody, *toBody, transATop, transBTop, true);

    return slider;
}

tgModel* tgPrismaticInfo::createModel(tgWorld& world)
{  
    tgNode startNode = this->getFrom();
    tgNode endNode = this->getTo();
    
    btVector3 buildVec = (endNode - startNode);
    double m_startLength = buildVec.length();

//    m_config.m_minLength = m_startLength;

    btSliderConstraint* slider = createSlider();

    tgPrismatic* prism = new tgPrismatic(slider, getTags(), m_config);
    prism->setup(world);
    return prism;
} 

double tgPrismaticInfo::getMass()
{
    // @todo: add up the rigid bodies
    return 0;
}
