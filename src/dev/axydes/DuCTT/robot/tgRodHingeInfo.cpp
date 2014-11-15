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
 * @file tgRodHingeInfo.cpp
 * @brief Contains the definition of members of the class tgHinge. A prismatic actuator builder.
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgRodHingeInfo.h"

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "LinearMath/btTransform.h"

#include <iostream>

#include "tgcreator/tgNode.h"
#include "tgcreator/tgStructureInfo.h"

tgRodHingeInfo::tgRodHingeInfo(const tgRodHinge::Config& config) :
    m_config(config),
    tgConnectorInfo() 
{
}

tgRodHingeInfo::tgRodHingeInfo(const tgRodHinge::Config& config, tgTags tags) :
    m_config(config),
    tgConnectorInfo(tags)
{}

tgRodHingeInfo::tgRodHingeInfo(const tgRodHinge::Config& config, const tgPair& pair) :
    m_config(config),
    tgConnectorInfo(pair)
{}

tgRodHingeInfo::~tgRodHingeInfo()
{
}

tgConnectorInfo* tgRodHingeInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgRodHingeInfo(m_config, pair);
}

void tgRodHingeInfo::initConnector(tgWorld& world)
{
}

btVector3 tgRodHingeInfo::getRigidVector(bool isCompound, std::set<btVector3> fromNodes)
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

btHingeConstraint* tgRodHingeInfo::createHinge()
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

    //this is essential to getting the right axis of rotation
    if (m_config.m_axis == 0)
    {
        //this is good for right/front hinges
        hinge->setAxis(axisA);
    }
    else if (m_config.m_axis == 1)
    {
        //need something for back/left, rotated by 90 degrees
        axisB = crossVect.cross(fromVect).safeNormalize();
        hinge->setAxis(axisB);
    }

    hinge->setDbgDrawSize(btScalar(5.f));
    return hinge;
}

tgModel* tgRodHingeInfo::createModel(tgWorld& world)
{  
    btHingeConstraint* hingeC = createHinge();

    tgRodHinge* hinge = new tgRodHinge(hingeC, getTags(), m_config);
    hinge->setup(world);
    return hinge;
} 

double tgRodHingeInfo::getMass()
{
    // @todo: add up the rigid bodies
    return 0;
}
