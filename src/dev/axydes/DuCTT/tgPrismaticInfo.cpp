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

tgModel* tgPrismaticInfo::createModel(tgWorld& world)
{  
    tgNode startNode = this->getFrom();
    tgNode endNode = this->getTo();
    
    btVector3 buildVec = (endNode - startNode);
    double m_startLength = buildVec.length();

    m_stringConfig.stiffness = 1000;
    m_stringConfig.damping = 10;
//    m_config.m_minLength = m_startLength;

    btRigidBody* fromBody = getFromRigidBody();
    btRigidBody* toBody = getToRigidBody();

    btVector3 from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo(), 0);//m_config.rotation);
    btVector3 to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom(), 0);//m_config.rotation);


    tgPrismatic* prism = new tgPrismatic(fromBody, from, toBody, to, getTags(), m_config);
    buildModel(world, prism);
    return prism;
} 

const int tgPrismaticInfo::getSegments() const
{
    //@todo: make configurable
    return m_config.m_segments;
}

double tgPrismaticInfo::getMass()
{
    // @todo: add up the rigid bodies
    return 0;
}

void tgPrismaticInfo::buildModel(tgWorld& world, tgModel* prismatic)
{
    // get global from _ to - still need these is both places I think.
    tgNode startNode = this->getFrom();
    tgNode endNode = this->getTo();

    prismatic->setup(world);

    /**
    // Calculate the size of the rods
    double rodSize = m_config.m_minTotalLength / 2.0;

    btVector3 buildVec = (endNode - startNode);
    btVector3 offset = btVector3(0.1,0.1,0.1);
    tgNode midNode = tgNode((buildVec / 2.0),"mid");
    tgNode midNode1 = tgNode(midNode - offset,"mid1");
    tgNode midNode2 = tgNode(midNode + offset,"mid2");

    //super structure to the prismatic actuator's two rods
    tgStructure prism;

    tgStructure rod;
    rod.addNode(0,0,0); //Bottom
    rod.addNode(midNode);
    rod.addPair(0,1,"a rod");

    tgStructure* rod1 = new tgStructure(rod);
    rod1->addTags("rod v1");
    rod1->move(startNode);
    prism.addChild(rod1);

    tgStructure* rod2 = new tgStructure(rod);
    rod2->addTags("rod v2");
    rod2->move(startNode + midNode);
    prism.addChild(rod2);

    std::cout << "Displaying all nodes:" << std::endl;
    std::cout << "From: " << startNode << ", To: " << endNode << std::endl;
    std::cout << "Rod1: " << rod1->getNodes()[0] << ", " << rod1->getNodes()[1] << std::endl;
    std::cout << "Rod2: " << rod2->getNodes()[0] << ", " << rod2->getNodes()[1] << std::endl;

    std::vector<tgStructure*> children = prism.getChildren();
    tgNodes n1 = children[0]->getNodes();
    tgNodes n2 = children[1]->getNodes();
//    prism.addPair(n1[1], n2[0], "anchor seg");
//    prism.addPair(n1[1], n2[0], "muscle seg");

    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(m_config.m_rodConfig));
    spec.addBuilder("muscle", new tgLinearStringInfo(m_stringConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(prism, spec);

    // Use the structureInfo to build our model
    structureInfo.buildInto(*prismatic, world);

    //not needed?
//    prismatic->setup(world);
    /**/

#if(0)
    // Debug printing
    std::cout << "StructureInfo:" << std::endl;
    std::cout << structureInfo << std::endl;

    std::cout << "Model: " << std::endl;
    std::cout << *prismatic << std::endl;
#endif
}
