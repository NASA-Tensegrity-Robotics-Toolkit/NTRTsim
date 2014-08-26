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
 * @brief Contains the definition of members of the class tgHinge. A Hinge actuator builder.
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgHingeInfo.h"


tgHingeInfo::tgHingeInfo(const tgHinge::Config& config) :
    m_config(config),
    tgConnectorInfo() 
{
}

tgHingeInfo::tgHingeInfo(const tgHinge::Config& config1, tgTags tags) :
    m_config(config),
    tgConnectorInfo(tags)
{}

tgHingeInfo::tgHingeInfo(const tgHinge::Config& config1, const tgPair& pair) :
    m_config(config),
    tgConnectorInfo(pair)
{}

tgConnectorInfo* tgHingeInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgHingeInfo(m_config, pair);
}

void tgHingeInfo::initConnector(tgWorld& world)
{
    // Anything to do here?
}

tgModel* tgHingeInfo::createModel(tgWorld& world)
{  
    tgNode startNode = this->getFrom();
    tgNode endNode = this->getTo();
    
    btVector3 buildVec = (endNode - startNode);
    m_startLength = buildVec.length();

    tgNode midNode = tgNode(startNode + (buildVec / 2.0),"mid");

    buildModel(world, new tgModel());
    return rod1;
} 

const int tgHingeInfo::getSegments() const
{
    //@todo: make configurable
    return 2;
}

double tgHingeInfo::getMass()
{
    // @todo: add up the rigid bodies
    return 0;
}

void tgHingeInfo::buildModel(tgWorld& world, tgModel* model)
{
    // get global from _ to - still need these is both places I think.
    tgNode startNode = this->getFrom();
    tgNode endNode = this->getTo();

}
