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
 * @file tgHingeInfo.cpp
 * @brief Contains the definition of members of the class tgHinge. A prismatic actuator builder.
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgHingeInfo.h"

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"

#include <iostream>

#include "tgcreator/tgNode.h"
#include "tgcreator/tgStructureInfo.h"

tgHingeInfo::tgHingeInfo(const tgHinge::Config& config) :
    m_config(config),
    tgConnectorInfo() 
{
}

tgHingeInfo::tgHingeInfo(const tgHinge::Config& config, tgTags tags) :
    m_config(config),
    tgConnectorInfo(tags)
{}

tgHingeInfo::tgHingeInfo(const tgHinge::Config& config, const tgPair& pair) :
    m_config(config),
    tgConnectorInfo(pair)
{}

tgHingeInfo::~tgHingeInfo()
{
}

tgConnectorInfo* tgHingeInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgHingeInfo(m_config, pair);
}

void tgHingeInfo::initConnector(tgWorld& world)
{
}

btHingeConstraint* tgHingeInfo::createHinge()
{
    btRigidBody* fromBody = getFromRigidBody();
    btVector3 from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo(), 0);
    btTransform transATop;
    transATop.setIdentity();
    transATop.setOrigin(fromBody->getWorldTransform().inverse() * from);
    transATop.setRotation(btQuaternion(btVector3(0,0,1),M_PI/2));

    btRigidBody* toBody = getToRigidBody();
    btVector3 to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom(), 0);
    btTransform transBTop;
    transBTop.setIdentity();
    transBTop.setOrigin(toBody->getWorldTransform().inverse() * to);
    transBTop.setRotation(btQuaternion(btVector3(0,0,1),M_PI/2));

    btHingeConstraint* slider = new btHingeConstraint(*fromBody, transATop);

    return slider;
}

tgModel* tgHingeInfo::createModel(tgWorld& world)
{  
    tgNode startNode = this->getFrom();
    tgNode endNode = this->getTo();
    
    btVector3 buildVec = (endNode - startNode);
    double m_startLength = buildVec.length();

    btHingeConstraint* slider = createHinge();

    tgHinge* hinge = new tgHinge(slider, getTags(), m_config);
    hinge->setup(world);
    return hinge;
} 

double tgHingeInfo::getMass()
{
    // @todo: add up the rigid bodies
    return 0;
}
