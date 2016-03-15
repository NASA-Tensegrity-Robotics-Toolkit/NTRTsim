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
    std::cout << "AppNestedStructureTest" << std::endl;

    std::cout << "creating structure 'root'" << std::endl;
    tgStructure* root = new tgStructure();
    std::cout << "creating structure 'sub1'" << std::endl;
    tgStructure* sub1 = new tgStructure();
    std::cout << "creating structure 'sub2'" << std::endl;
    tgStructure* sub2 = new tgStructure();
    
    std::cout << "Adding nodes and pairs to the leaf" << std::endl;

    // Add some nodes and pairs to the leaf
    sub2->addNode(1,2,3);
    sub2->addNode(4,5,6);
    sub2->addPair(0,1);
    
    // Add the child to the middle parent
    sub1->addChild(sub2);
    
    // Add a copy of the child to the middle parent and move it

    tgStructure* sub3 = new tgStructure(*sub2);
    sub3->move(btVector3(0,4,0));
    
    sub1->addChild(sub3);
    
    std::cout << "sub 1:" << std::endl;
    std::cout << *sub1 << std::endl;
    
}
