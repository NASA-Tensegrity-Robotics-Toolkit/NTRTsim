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
 * @file DuCTTModel.cpp
 * @brief Contains the definition of the members of the class DuCTTModel.
 * $Id$
 */

// This module
#include "DuctStraightModel.h"
// This library
#include "core/tgBox.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>

/**
 * Anonomous namespace so we don't have to declare the config in
 * the header.
 */
namespace
{
    /**
     * Configuration parameters so they're easily accessable.
     * All parameters must be positive.
     //
     // see tgBaseString.h for a descripton of some of these rod parameters
     // (specifically, those related to the motor moving the strings.)
     //
     // NOTE that any parameter that depends on units of length will scale
     // with the current gravity scaling. E.g., with gravity as 981,
     // the length units below are in centimeters.
     //
     // Total mass of bars is about 1.5 kg.  Total
     */
    const struct Config
    {
        double ductHeight;
        double ductWidth;
        double distance; //amount of distance to extend
        double wallWidth;
        double friction;
        int axis; // which axis to extend along, defaults to y, 0=x, 1=y, 2=z
    } c =
   {
        34,
        34,
        100,
        0.5,
        1.0,
        1
   }
    ;
} // namespace

DuctStraightModel::DuctStraightModel() :
tgModel()
{
}

DuctStraightModel::~DuctStraightModel()
{
}

void DuctStraightModel::addNodesXAxis(tgStructure &s)
{
    s.addNode(0, 0, -c.ductHeight/2.0);
    s.addNode(c.distance, 0, -c.ductHeight/2.0);

    s.addNode(0, 0, c.ductHeight/2.0);
    s.addNode(c.distance, 0, c.ductHeight/2.0);

    s.addNode(0, -c.ductWidth/2.0, 0);
    s.addNode(c.distance, -c.ductWidth/2.0, 0);

    s.addNode(0, c.ductWidth/2.0, 0);
    s.addNode(c.distance, c.ductWidth/2.0, 0);
}

void DuctStraightModel::addNodesYAxis(tgStructure &s)
{
    s.addNode(0, 0, -c.ductHeight/2.0);
    s.addNode(0, c.distance, -c.ductHeight/2.0);

    s.addNode(0, 0, c.ductHeight/2.0);
    s.addNode(0, c.distance, c.ductHeight/2.0);

    s.addNode(-c.ductWidth/2.0, 0, 0);
    s.addNode(-c.ductWidth/2.0, c.distance, 0);

    s.addNode(c.ductWidth/2.0, 0, 0);
    s.addNode(c.ductWidth/2.0, c.distance, 0);
}

void DuctStraightModel::addNodesZAxis(tgStructure &s)
{
    s.addNode(0, -c.ductHeight/2.0, 0);
    s.addNode(0, -c.ductHeight/2.0, c.distance);

    s.addNode(0, c.ductHeight/2.0, 0);
    s.addNode(0, c.ductHeight/2.0, c.distance);

    s.addNode(-c.ductWidth/2.0, 0, 0);
    s.addNode(-c.ductWidth/2.0, 0, c.distance);

    s.addNode(c.ductWidth/2.0, 0, 0);
    s.addNode(c.ductWidth/2.0, 0, c.distance);
}

void DuctStraightModel::addNodes(tgStructure &s)
{
    switch(c.axis)
    {
    case 0:
        addNodesXAxis(s);
        break;
    case 1:
        addNodesYAxis(s);
        break;
    case 2:
        addNodesZAxis(s);
        break;
    default:
        addNodesYAxis(s);
        break;
    }
}

void DuctStraightModel::addBoxes(tgStructure &s)
{
    s.addPair(0,1, "box");
    s.addPair(2,3, "box");
    s.addPair(4,5, "box2");
    s.addPair(6,7, "box2");
}

void DuctStraightModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    const tgBox::Config boxConfig(c.ductWidth/2.0, c.wallWidth, 0, c.friction);
    const tgBox::Config boxConfig2(c.wallWidth, c.ductHeight/2.0, 0, c.friction);
    
    // Create a structure that will hold the details of this model
    tgStructure s;

    addNodes(s);
    addBoxes(s);

    // Move the structure so it doesn't start in the ground
    if (c.axis == 0)
    {
        s.move(btVector3(0, c.ductWidth/2.0+1, 0));
    }
    else if (c.axis == 2)
    {
        s.move(btVector3(0, c.ductHeight/2.0+1, 0));
    }
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("box", new tgBoxInfo(boxConfig));
    spec.addBuilder("box2", new tgBoxInfo(boxConfig2));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);
    
    // Actually setup the children
    tgModel::setup(world);
}

void DuctStraightModel::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        tgModel::step(dt);  // Step any children
    }
}

void DuctStraightModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}
    
void DuctStraightModel::teardown()
{
    tgModel::teardown();
}
