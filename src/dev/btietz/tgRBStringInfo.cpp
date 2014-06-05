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
 * @file tgRBStringInfo.cpp
 * @brief Contains the definition of members of the class tgRBString. A 
 * string with small rigid bodies to create contact dynamics.
 * @author Brian Tietz
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgRBStringInfo.h"

#include "btBulletDynamicsCommon.h"
#include <iostream>

#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgConnectorInfo.h"
#include "tgcreator/tgLinearStringInfo.h"
//#include "../tgRigidAutoCompound.h"
#include "tgcreator/tgBuildSpec.h"

#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgNode.h"
#include "tgcreator/tgPair.h"

#include "core/tgBaseString.h"
#include "core/tgRod.h"
#include "tgcreator/tgUtil.h"
#include "core/tgString.h"

tgRBStringInfo::tgRBStringInfo(const tgRBString::Config& config) :
    m_config(config),
    tgConnectorInfo() 
{
}

tgRBStringInfo::tgRBStringInfo(const tgRBString::Config& config, tgTags tags) : 
    m_config(config),
    tgConnectorInfo(tags)
{}

tgRBStringInfo::tgRBStringInfo(const tgRBString::Config& config, const tgPair& pair) :
    m_config(config),
    tgConnectorInfo(pair)
{}

tgModel* tgRBStringInfo::createModel(tgWorld& world)
{  
    tgNode startNode = this->getFrom();
    tgNode endNode = this->getTo();
    
    btVector3 buildVec = (endNode - startNode);
    double startLength = buildVec.length();
    
    tgRBString* thisString = new tgRBString(getTags(), m_config, startLength);
    buildModel(world, thisString);
    return thisString;
} 

void tgRBStringInfo::buildModel(tgWorld& world, tgModel* thisString)
{
    // get global from _ to - still need these is both places I think.
    tgNode startNode = this->getFrom();
    tgNode endNode = this->getTo();
    
    // FIRST create intermedate body
    
    // todo - figure out a way to change only the one parameter with less code
    const double stiffness = m_config.m_stringConfig.stiffness;
    const double damping = m_config.m_stringConfig.damping;
    tgLinearString::Config muscleConfig1(stiffness, damping, false, -M_PI / 2.0);
    tgLinearString::Config muscleConfig2(stiffness, damping, false, M_PI / 2.0);
    tgLinearString::Config muscleConfig3(stiffness, damping, false, M_PI);
    tgLinearString::Config muscleConfig4(stiffness, damping, false, 0);
    
    // Calculate the size of the rods
    double v_size = m_config.m_minTotalLength / (double) m_config.m_segments;
    
    // tgNode is a subclass, so this should be fine
    btVector3 buildVec = (endNode - startNode);
    double startLength = buildVec.length();
    
    double spacing = (startLength - v_size / 2.0) / (double) m_config.m_segments;
    // After this buildVec stays normalized.
    btVector3 offset = buildVec.normalize() * spacing; 
    
    // Create the sub rods
    tgStructure strRod;
    
    strRod.addNode(0,0,0);  // Bottom
    tgNode secNode(v_size * buildVec); //Fancy upcast
    strRod.addNode(secNode); // Top
    
    strRod.addPair(0,1, "a rod");

    // Move the first one into position
    strRod.move(startNode);

    // Create our sub-structure string
    tgStructure snake;
    
    for(int i = 0; i < m_config.m_segments; i++) {
        // Destructor is called when tgStructure is destroyed (end of this method)
        tgStructure* t = new tgStructure(strRod);
        t->addTags(tgString("srod num", i + 1));
        t->move((i + 1)*offset);
        snake.addChild(t); // Add a child to the snake
    }

    // Add muscles that connect the segments
    std::vector<tgStructure*> children = snake.getChildren();
    for(int i = 1; i < children.size(); i++) {
        tgNodes n0 = children[i-1]->getNodes();
        tgNodes n1 = children[i]->getNodes();
        
        snake.addPair(n0[1], n1[0], tgString("muscle v1 seg", i) + tgString(" seg", i));
        snake.addPair(n0[1], n1[0], tgString("muscle v2 seg", i) + tgString(" seg", i));
        snake.addPair(n0[1], n1[0], tgString("muscle v3 seg", i) + tgString(" seg", i));
        snake.addPair(n0[1], n1[0], tgString("muscle v4 seg", i) + tgString(" seg", i));
    }
    
        // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(m_config.m_rodConfig));
    
    spec.addBuilder("muscle v1", new tgLinearStringInfo(muscleConfig1));
    spec.addBuilder("muscle v2", new tgLinearStringInfo(muscleConfig2));
    spec.addBuilder("muscle v3", new tgLinearStringInfo(muscleConfig3));
    spec.addBuilder("muscle v4", new tgLinearStringInfo(muscleConfig4));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);
    
    

    // Use the structureInfo to build our model
    structureInfo.buildInto(*thisString, world);
    
    /* Now that the strings sub-structure has been created, attach it
    * to the super-structure, again using four strings
    * Very manual from here, since this is the only place we have all
    * of the necessary pointers 
    */ 
    
    // We have to delete these
    std::vector<tgPair*> linkerPairs;
    
    // "From" side:
    tgNodes n0 = children[0]->getNodes();
    // Saves code to only have one - but might mess up the tags a little
    linkerPairs.push_back(new tgPair(startNode, n0[0], tgString("anchor seg", 0) + tgString(" seg", 1)));
    
    // "To" side:
    std::size_t cldend = children.size() - 1;
    tgNodes n1 = children[cldend]->getNodes();
    linkerPairs.push_back(new tgPair(n1[1], endNode, tgString("anchor seg", cldend - 1) + tgString(" seg", cldend)));
    
    // Perhaps overkill...
    // These pointers will be deleted by tgStructure
    std::vector <tgRigidInfo*> allRigids(structureInfo.getAllRigids());
    allRigids.push_back(getToRigidInfo());
    allRigids.push_back(getFromRigidInfo());
    
    // We have to delete these
    std::vector <tgConnectorInfo* > linkerInfo;
    
    // Can this part be done in a loop easily?
    linkerInfo.push_back(new tgLinearStringInfo(muscleConfig1, *linkerPairs[0]));
    linkerInfo.push_back(new tgLinearStringInfo(muscleConfig1, *linkerPairs[1]));
    
    linkerInfo.push_back(new tgLinearStringInfo(muscleConfig2, *linkerPairs[0]));
    linkerInfo.push_back(new tgLinearStringInfo(muscleConfig2, *linkerPairs[1]));
    
    linkerInfo.push_back(new tgLinearStringInfo(muscleConfig3, *linkerPairs[0]));
    linkerInfo.push_back(new tgLinearStringInfo(muscleConfig3, *linkerPairs[1]));
    
    linkerInfo.push_back(new tgLinearStringInfo(muscleConfig4, *linkerPairs[0]));
    linkerInfo.push_back(new tgLinearStringInfo(muscleConfig4, *linkerPairs[1]));
    
    for (std::size_t i = 0; i < linkerInfo.size(); i++)
    {
        linkerInfo[i]->chooseRigids(allRigids);
        linkerInfo[i]->initConnector(world);
        
        tgModel* m = linkerInfo[i]->createModel(world);
        m->setTags(linkerInfo[i]->getTags());
        if(m != 0) {            
            thisString->addChild(m);
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
    
#if (0)
    // Debug printing
    std::cout << "StructureInfo:" << std::endl;
    std::cout << structureInfo << std::endl;

    std::cout << "Model: " << std::endl;
    std::cout << *thisString << std::endl;
#endif
        
}
