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

btVector3 tgHingeInfo::getRigidVector(bool isCompound, std::set<btVector3> fromNodes)
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

btHingeConstraint* tgHingeInfo::createHinge()
{
    btVector3 axisA = btVector3(0,0,1);
    switch (m_config.m_axisFrom)
    {
    case 0:
        axisA = btVector3(1,0,0);
        break;
    case 1:
        axisA = btVector3(0,1,0);
        break;
    case 2:
        axisA = btVector3(0,0,1);
        break;
    }
    btVector3 axisB = btVector3(0,0,1);
    switch (m_config.m_axisTo)
    {
    case 0:
        axisB = btVector3(1,0,0);
        break;
    case 1:
        axisB = btVector3(0,1,0);
        break;
    case 2:
        axisB = btVector3(0,0,1);
        break;
    }

    std::set<btVector3> fromNodes = getFromRigidInfo()->getContainedNodes();
    btRigidBody* fromBody = getFromRigidBody();
    btVector3 from = getFrom();

    std::set<btVector3> toNodes = getToRigidInfo()->getContainedNodes();
    btRigidBody* toBody = getToRigidBody();
    btVector3 to = getTo();

    if (from != to)
    {
//        from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo(), 0);
//        to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom(), 0);
    }

    btVector3 fromVect = getRigidVector(getFromRigidInfo()->isCompound(), fromNodes);
    btVector3 toVect = getRigidVector(getToRigidInfo()->isCompound(), toNodes);
//    btVector3 fromVect = fromBody->getOrientation().getAxis();
//    btVector3 toVect = toBody->getOrientation().getAxis();

    std::cout << fromNodes.size() << " " << fromVect << " " << from << std::endl;
    std::cout << toNodes.size() << " " << toVect << " " << to << std::endl;

    btVector3 crossVect = toVect.cross(fromVect).safeNormalize();
    std::cout << crossVect << std::endl;
    axisA = crossVect;

    btVector3 crossVect2 = fromVect.cross(toVect).safeNormalize();
    std::cout << crossVect2 << std::endl;
    axisB = crossVect2;

    btVector3 oriA;
//    oriA = to;
//    oriA = from;
    oriA = from + 0.01*fromVect;
//    oriA = from - ((to - from) / 2.0);
//    oriA = fromBody->getWorldTransform().inverse() * from;
//    oriA = fromBody->getWorldTransform().inverse() * to;
//    oriA = fromBody->getWorldTransform().inverse() * fromBody->getCenterOfMassTransform().getOrigin();
    oriA = fromBody->getWorldTransform().inverse() * oriA;

    btVector3 oriB;
//    oriB = from;
//    oriB = to;
    oriB = to + 0.01*toVect;
//    oriB = to + ((from - to) / 2.0);
//    oriB = toBody->getWorldTransform().inverse() * to;
//    oriB = toBody->getWorldTransform().inverse() * from;
//    oriB = toBody->getWorldTransform().inverse() * toBody->getCenterOfMassTransform().getOrigin();
    oriB = toBody->getWorldTransform().inverse() * oriB;

//    std::cout << oriA << oriB << std::endl;
    std::cout << std::endl;

    btHingeConstraint* hinge = new btHingeConstraint(*fromBody, *toBody, oriA, oriB, axisA, axisB);
//    btHingeConstraint* hinge = new btHingeConstraint(*fromBody, *toBody, oriA, oriB, axisA, axisA);

    hinge->setDbgDrawSize(btScalar(5.f));
    return hinge;
}

tgModel* tgHingeInfo::createModel(tgWorld& world)
{  
    btHingeConstraint* hingeC = createHinge();

    tgHinge* hinge = new tgHinge(hingeC, getTags(), m_config);
    hinge->setup(world);
    return hinge;
} 

double tgHingeInfo::getMass()
{
    // @todo: add up the rigid bodies
    return 0;
}
