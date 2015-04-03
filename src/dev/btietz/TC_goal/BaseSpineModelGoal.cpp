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
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <algorithm> // std::fill
#include <iostream>
#include <map>
#include <set>

BaseSpineModelGoal::BaseSpineModelGoal(int segments) : 
    BaseSpineModelLearning(segments) 
{
}

BaseSpineModelGoal::~BaseSpineModelGoal()
{
}

void BaseSpineModelGoal::setup(tgWorld& world)
{

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

