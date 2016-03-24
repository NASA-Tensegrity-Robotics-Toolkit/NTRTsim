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
    
    tgStructure* sub0 = new tgStructure("sub0");
    // sub0->setName("sub0");
    
    tgStructure* sub1_1 = new tgStructure("sub1_1");
    //sub1_1->setName("s1_1");
    
    sub1_1->addNode(1,2,3,"one two three");
    sub1_1->addNode(4,5,6, "four five six");
    sub1_1->addPair(0,1,"one two");
    
    tgStructure* sub1_2 = new tgStructure(*sub1_1);
    sub1_2->setTags(tgTags("sub1_2"));
    sub1_2->move(btVector3(4,4,4));
    //sub1_2->setName("s1_2");
    
    sub0->addChild(sub1_1);
    sub0->addChild(sub1_2);
    
    tgStructure* root = new tgStructure();
    //root->setName("root");
    root->addChild(sub0);
    root->setTags(tgTags("root"));

    tgStructure* sub1 = new tgStructure(*sub0);
    //sub1->setName("sub1");
    sub1->move(btVector3(10,10,10)); // PROBLEM: This moves things in sub0...
    root->addChild(sub1);
    
    std::cout << *root << std::endl;
    
}
