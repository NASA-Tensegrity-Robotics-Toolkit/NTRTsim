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
 * @file AppPrismModel.cpp
 * @brief Contains the definition function main() for the Three strut
 * tensegrity prism example application
 * @author Ryan Adams
 * $Id$
 */

// This application

// This library
#include "tgcreator/tgNode.h"
#include "tgcreator/tgNodes.h"
#include "tgcreator/tgPair.h"
#include "tgcreator/tgStructure.h"
// The C++ Standard Library
#include <iostream>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "AppTgNamesTest" << std::endl;
    tgNode node = tgNode(1,2,3,"these are tags", "node1");
    std::cout << "node print test: " << node << std::endl;
    
    tgNodes nodes;
    
    nodes.addNode(node);
    
    tgNodes nodes2 = tgNodes(nodes);
    nodes2.move(btVector3(2,2,2));
    
    std::cout << "nodes: " << nodes << std::endl;
    std::cout << "nodes2: " << nodes2 << std::endl;
    
    tgStructure* s = new tgStructure();
    s->addNode(1,2,3,"");
    s->addNode(node);
    
    s->setName("test structure 1");
    
    tgStructure* p = new tgStructure();
    p->addChild(s);
    s->setName("after change");
    p->addChild(s);
    
    //tgStructure s = tgStructure();
    //s.addNode(2,3,4,"more tags");  // This works, but s.addNode(node) does not (malloc error)
    //s.addNode(node);
    // s.addNode(tgNode(1,2,3,"these are tags", "node1"));
    
    std::cout << "structure print test: " << s << std::endl;
    std::cout << "node access by name test: " << nodes["node1"] << std::endl;
    
    std::cout << std::endl;
    std::cout << "Structure copy test:" << std::endl;
    std::cout << *p << std::endl;
}
