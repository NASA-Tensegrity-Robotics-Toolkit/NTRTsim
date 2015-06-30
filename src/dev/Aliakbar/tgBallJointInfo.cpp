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
 * @file tgBallJointInfo.cpp
 * @brief Contains the definition of members of the class tgBallJoint. A prismatic actuator builder.
 * @author Aliakbar Toghyan
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgBallJointInfo.h"

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "LinearMath/btTransform.h"

#include <iostream>

#include "tgcreator/tgNode.h"
#include "tgcreator/tgStructureInfo.h"

tgBallJointInfo::tgBallJointInfo(const tgBallJoint::Config& config) :
    m_config(config),
    tgConnectorInfo() 
{
}

tgBallJointInfo::tgBallJointInfo(const tgBallJoint::Config& config, tgTags tags) :
    m_config(config),
    tgConnectorInfo(tags)
{}

tgBallJointInfo::tgBallJointInfo(const tgBallJoint::Config& config, const tgPair& pair) :
    m_config(config),
    tgConnectorInfo(pair)
{}

tgBallJointInfo::~tgBallJointInfo()
{
}

tgConnectorInfo* tgBallJointInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgBallJointInfo(m_config, pair);
}

void tgBallJointInfo::initConnector(tgWorld& world)
{
}

btVector3 tgBallJointInfo::getRigidVector(bool isCompound, std::set<btVector3> fromNodes)
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

btPoint2PointConstraint* tgBallJointInfo::createBallJoint()
{
    std::set<btVector3> fromNodes = getFromRigidInfo()->getContainedNodes();
    btRigidBody* fromBody = getFromRigidBody();
    btVector3 from = getFrom();

    std::set<btVector3> toNodes = getToRigidInfo()->getContainedNodes();
    btRigidBody* toBody = getToRigidBody();
    btVector3 to = getTo();

    btVector3 fromVect = getRigidVector(getFromRigidInfo()->isCompound(), fromNodes);
    btVector3 toVect = getRigidVector(getToRigidInfo()->isCompound(), toNodes);


    btVector3 oriA;
    oriA = from - 0.05*fromVect;
    oriA = fromBody->getWorldTransform().inverse() * oriA;

    btVector3 oriB;
    oriB = to - 0.05*toVect;
    oriB = toBody->getWorldTransform().inverse() * oriB;

    btPoint2PointConstraint* balljoint = new btPoint2PointConstraint(*fromBody, *toBody, oriA, oriB);

    

    balljoint->setDbgDrawSize(btScalar(5.f));
    return balljoint;
}

tgModel* tgBallJointInfo::createModel(tgWorld& world)
{  
    btPoint2PointConstraint* balljointC = createBallJoint();

    tgBallJoint* balljoint = new tgBallJoint(balljointC, getTags(), m_config);
    balljoint->setup(world);
    return balljoint;
} 

double tgBallJointInfo::getMass()
{
    // @todo: add up the rigid bodies
    return 0;
}