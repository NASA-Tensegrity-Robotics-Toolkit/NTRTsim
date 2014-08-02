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

tgConnectorInfo* tgPrismaticInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgPrismaticInfo(m_config, pair);
}

void tgPrismaticInfo::initConnector(tgWorld& world)
{
    // Anything to do here?
}

tgModel* tgPrismaticInfo::createModel(tgWorld& world)
{  
    tgNode startNode = this->getFrom();
    tgNode endNode = this->getTo();
    
    btVector3 buildVec = (endNode - startNode);
    double m_startLength = buildVec.length();

    tgNode midNode = tgNode(startNode + (buildVec / 2.0),"mid");

    m_config.m_minTotalLength = m_startLength;

    tgPrismatic* prism = new tgPrismatic(getTags(), m_config);
    buildModel(world, prism);
    return prism;
} 

const int tgPrismaticInfo::getSegments() const
{
    //@todo: make configurable
    return 2;
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
    rod.addNode(midNode1);
    rod.addPair(0,1,"a rod");

    tgStructure* rod1 = new tgStructure(rod);
    rod1->addTags("rod v1");
    rod1->move(startNode + offset);
    prism.addChild(rod1);

    tgStructure* rod2 = new tgStructure(rod);
    rod2->addTags("rod v2");
    rod2->move(startNode + midNode2);
    prism.addChild(rod2);

    std::vector<tgStructure*> children = prism.getChildren();
    tgNodes n1 = children[0]->getNodes();
    tgNodes n2 = children[1]->getNodes();
    prism.addPair(n1[1], n2[0], "muscle seg");

    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(m_config.m_rod1Config));
    spec.addBuilder("muscle", new tgLinearStringInfo(m_stringConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(prism, spec);

    std::cout << "tgPrismaticInfo::buildModel() 7" << std::endl;
    // Use the structureInfo to build our model
    structureInfo.buildInto(*prismatic, world);

    /* Now that the sub-structure has been created, attach it
    * to the super-structure via hard links?
    * Very manual from here, since this is the only place we have all
    * of the necessary pointers
    */
    // We have to delete these
    std::vector<tgPair*> linkerPairs;

    // "From" side:
    linkerPairs.push_back(new tgPair(startNode, n1[0], tgString("anchor seg",1)));
    // "To" side:
    linkerPairs.push_back(new tgPair(n2[1], endNode, tgString("anchor seg",2)));

    // Perhaps overkill...
    // These pointers will be deleted by tgStructure
    std::vector <tgRigidInfo*> allRigids(structureInfo.getAllRigids());
    std::cout << "tgPrismaticInfo num rigids: " << allRigids.size() << std::endl;
    allRigids.push_back(getToRigidInfo());
    allRigids.push_back(getFromRigidInfo());

    // We have to delete these
    std::vector <tgConnectorInfo* > linkerInfo;

    // Can this part be done in a loop easily?
//    linkerInfo.push_back(new tgLinearStringInfo(m_stringConfig, *linkerPairs[0]));
//    linkerInfo.push_back(new tgLinearStringInfo(m_stringConfig, *linkerPairs[1]));

    for (std::size_t i = 0; i < linkerInfo.size(); i++)
    {
        linkerInfo[i]->chooseRigids(allRigids);
        linkerInfo[i]->initConnector(world);

        tgModel* m = linkerInfo[i]->createModel(world);
        m->setTags(linkerInfo[i]->getTags());
        if(m != 0) {
            prismatic->addChild(m);
        }
    }

    // Clean up - see if there's a way to do this with references instead?
    for (std::size_t i = 0; i < linkerInfo.size(); i++)
    {
        delete linkerInfo[i];
    }
    linkerInfo.clear();

    for (std::size_t i = 0; i < linkerPairs.size(); i++)
    {
        delete linkerPairs[i];
    }
    linkerPairs.clear();

    /**
    double m_startLength = buildVec.length();
    btVector3 offset = buildVec.normalize();
    tgStructure rod;
    rod.addNode(0,0,0); //Bottom
    rod.addNode(midNode);
    rod.addPair(0,1,"a rod");

    //1st rod
    tgStructure* rod1 = new tgStructure(rod);

    //Move the first one into position
    rod1->addTags("rod v1");
    rod1->move(startNode);
    prism.addChild(rod1);

    //second rod
    tgStructure* rod2 = new tgStructure(rod);
    rod2->addTags("rod v2");
    rod2->move(startNode + midNode);
    prism.addChild(rod2);

    std::vector<tgStructure*> children = prism.getChildren();
    tgNodes n1 = children[0]->getNodes();
    tgNodes n2 = children[1]->getNodes();
    prism.addPair(n1[1], n2[0], "muscle seg");

    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(m_config.m_rod1Config));
//    spec.addBuilder("rod v1", new tgRodInfo(m_config.m_rod1Config));
//    spec.addBuilder("rod v2", new tgRodInfo(m_config.m_rod2Config));
    spec.addBuilder("muscle", new tgLinearStringInfo(m_stringConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(prism, spec);

    std::cout << "tgPrismaticInfo::buildModel() 7" << std::endl;
    // Use the structureInfo to build our model
    structureInfo.buildInto(*prismatic, world);

    /* Now that the sub-structure has been created, attach it
    * to the super-structure
    * Very manual from here, since this is the only place we have all
    * of the necessary pointers
    *
    // We have to delete these
    std::vector<tgPair*> linkerPairs;

    // "From" side:
    linkerPairs.push_back(new tgPair(startNode, n1[0], tgString("anchor seg",1)));
    // "To" side:
    linkerPairs.push_back(new tgPair(n2[1], endNode, tgString("anchor seg",2)));

    // Perhaps overkill...
    // These pointers will be deleted by tgStructure
    std::vector <tgRigidInfo*> allRigids(structureInfo.getAllRigids());
    std::cout << "tgPrismaticInfo num rigids: " << allRigids.size() << std::endl;
    allRigids.push_back(getToRigidInfo());
    allRigids.push_back(getFromRigidInfo());

    // We have to delete these
    std::vector <tgConnectorInfo* > linkerInfo;

    // Can this part be done in a loop easily?
    linkerInfo.push_back(new tgLinearStringInfo(m_stringConfig, *linkerPairs[0]));
    linkerInfo.push_back(new tgLinearStringInfo(m_stringConfig, *linkerPairs[1]));

    for (std::size_t i = 0; i < linkerInfo.size(); i++)
    {
        linkerInfo[i]->chooseRigids(allRigids);
        linkerInfo[i]->initConnector(world);

        tgModel* m = linkerInfo[i]->createModel(world);
        m->setTags(linkerInfo[i]->getTags());
        if(m != 0) {
            prismatic->addChild(m);
        }
    }

    // Clean up - see if there's a way to do this with references instead?
    for (std::size_t i = 0; i < linkerInfo.size(); i++)
    {
        delete linkerInfo[i];
    }
    linkerInfo.clear();

    for (std::size_t i = 0; i < linkerPairs.size(); i++)
    {
        delete linkerPairs[i];
    }
    linkerPairs.clear();
    /**/

    std::cout << "tgPrismaticInfo::buildModel() 8" << std::endl;
    //not needed?
//    prismatic->setup(world);

#if(0)
    // Debug printing
    std::cout << "StructureInfo:" << std::endl;
    std::cout << structureInfo << std::endl;

    std::cout << "Model: " << std::endl;
    std::cout << *prismatic << std::endl;
#endif
}
