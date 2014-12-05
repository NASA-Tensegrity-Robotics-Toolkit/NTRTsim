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
 * @file tgDuCTTHingeInfo.cpp
 * @brief Contains the definition of members of the class tgHinge. A prismatic actuator builder.
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgDuCTTHingeInfo.h"

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "LinearMath/btTransform.h"

#include <iostream>

#include "tgcreator/tgNode.h"
#include "tgcreator/tgStructureInfo.h"

tgDuCTTHingeInfo::tgDuCTTHingeInfo(const tgDuCTTHinge::Config& config) :
    m_config(config),
    tgConnectorInfo() 
{
}

tgDuCTTHingeInfo::tgDuCTTHingeInfo(const tgDuCTTHinge::Config& config, tgTags tags) :
    m_config(config),
    tgConnectorInfo(tags)
{}

tgDuCTTHingeInfo::tgDuCTTHingeInfo(const tgDuCTTHinge::Config& config, const tgPair& pair) :
    m_config(config),
    tgConnectorInfo(pair)
{}

tgDuCTTHingeInfo::~tgDuCTTHingeInfo()
{
}

tgConnectorInfo* tgDuCTTHingeInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgDuCTTHingeInfo(m_config, pair);
}

void tgDuCTTHingeInfo::initConnector(tgWorld& world)
{
}

btVector3 tgDuCTTHingeInfo::getRigidVector(bool isCompound, std::set<btVector3> fromNodes)
{
    btVector3 vect;
    if (!isCompound && fromNodes.size() == 2)
    {
        std::set<btVector3>::iterator itr;
        int i=0;
        for (itr = fromNodes.begin(); itr != fromNodes.end(); itr++)
        {
            btVector3 temp = *itr;
            switch(i)
            {
            case 0:
                vect += temp;
                break;
            case 1:
                vect -= temp;
                break;
            }
            i++;
        }
    }
    return vect;
}

btHingeConstraint* tgDuCTTHingeInfo::createHinge()
{
    std::set<btVector3> fromNodes = getFromRigidInfo()->getContainedNodes();
    btRigidBody* fromBody = getFromRigidBody();
    btVector3 from = getFrom();

    std::set<btVector3> toNodes = getToRigidInfo()->getContainedNodes();
    btRigidBody* toBody = getToRigidBody();
    btVector3 to = getTo();

    btVector3 fromVect = getRigidVector(getFromRigidInfo()->isCompound(), fromNodes);
    btVector3 toVect = getRigidVector(getToRigidInfo()->isCompound(), toNodes);

    btVector3 crossVect = toVect.cross(fromVect).safeNormalize();
    btVector3 axisA = crossVect;

    btVector3 crossVect2 = fromVect.cross(toVect).safeNormalize();
    btVector3 axisB = crossVect2;

    btVector3 oriA;
    oriA = from + 0.001*fromVect;
    oriA = fromBody->getWorldTransform().inverse() * oriA;

    btVector3 oriB;
    oriB = to + 0.001*toVect;
    oriB = toBody->getWorldTransform().inverse() * oriB;

    btHingeConstraint* hinge = new btHingeConstraint(*fromBody, *toBody, oriA, oriB, axisA, axisB);

    hinge->setAxis(m_config.m_axis);

    hinge->setDbgDrawSize(btScalar(5.f));
    return hinge;
}

tgModel* tgDuCTTHingeInfo::createModel(tgWorld& world)
{  
    btHingeConstraint* hingeC = createHinge();

    tgDuCTTHinge* hinge = new tgDuCTTHinge(hingeC, getTags(), m_config);
    hinge->setup(world);
    return hinge;
} 

double tgDuCTTHingeInfo::getMass()
{
    // @todo: add up the rigid bodies
    return 0;
}
