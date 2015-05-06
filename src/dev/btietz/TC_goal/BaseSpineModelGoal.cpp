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
 * @file BaseSpineModelGoal.cpp
 * @brief Implementing the tetrahedral complex spine inspired by Tom Flemons
 * @author Brian Mirletz
 * @date April 2015
 * @version 1.1.0
 * $Id$
 */

// This module
#include "BaseSpineModelGoal.h"
// This library
#include "core/tgBox.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <algorithm> // std::fill
#include <iostream>
#include <map>
#include <set>

/// Rand seeding simular to the evolution classes. 
/// @todo should we make this common?
#ifdef _WIN32

//  Windows
#define rdtsc  __rdtsc

#else

//  For everything else
unsigned long long rdtsc(){
    unsigned int lo,hi;
    __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
    return ((unsigned long long)hi << 32) | lo;
}

#endif

BaseSpineModelGoal::BaseSpineModelGoal(int segments, double goalAngle) : 
    BaseSpineModelLearning(segments),
    m_goalAngle(goalAngle)
{
    srand(rdtsc());
}

BaseSpineModelGoal::~BaseSpineModelGoal()
{
}

void BaseSpineModelGoal::setup(tgWorld& world)
{
    
    // Create goal box in a new structure
#if (1)
    m_goalAngle = ((rand() / (double)RAND_MAX) - 0.5) * 3.1415 + 3.1415;
#endif // If we're resetting the simulation and want to change the angle    
    
    double xPos = 350 * sin(m_goalAngle);
    double zPos = 350 * cos(m_goalAngle);
    
    tgStructure goalBox;
    
    goalBox.addNode(xPos, 20.0, zPos);
    goalBox.addNode(xPos + 5.0, 20.0, zPos);
    
    goalBox.addPair(0, 1, "goalBox");
    
    // 1 by 1 by 1 box, fix when tgBoxInfo gets fixed
    const tgBox::Config boxConfig(10.0, 10.0);

    tgBuildSpec boxSpec;
    boxSpec.addBuilder("goalBox", new tgBoxInfo(boxConfig));
    
    tgStructureInfo goalStructureInfo(goalBox, boxSpec);
    
    goalStructureInfo.buildInto(*this, world);
    
    // A little sloppy, but I'm pretty confident there is only one
    m_goalBox = (find<tgBox>("goalBox"))[0];
    
    assert(m_goalBox != NULL);
    
    // Actually setup the children, notify controller
    BaseSpineModelLearning::setup(world);
}

void BaseSpineModelGoal::teardown()
{
    
    BaseSpineModelLearning::teardown();
      
}

void BaseSpineModelGoal::step(double dt)
{
    /* CPG update occurs in the controller so that we can decouple it
    * from the physics update
    */
    
    BaseSpineModelLearning::step(dt);  // Step any children
}

btVector3 BaseSpineModelGoal::goalBoxPosition() const
{
    return m_goalBox->centerOfMass();
}

