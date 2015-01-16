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

DuctStraightModel::Config::Config(
    double ductHeight,
    double ductWidth,
    double distance,
    double wallWidth,
    double friction,
    int axis,
    btVector3 startPos
    ) :
m_ductHeight(ductHeight),
m_ductWidth(ductWidth),
m_distance(distance),
m_wallWidth(wallWidth),
m_friction(friction),
m_axis(axis),
m_startPos(startPos)
{
}

DuctStraightModel::DuctStraightModel() :
    m_config(DuctStraightModel::Config()),
    tgModel()
{
}

DuctStraightModel::DuctStraightModel(DuctStraightModel::Config &config) :
    m_config(config),
    tgModel()
{
}

DuctStraightModel::~DuctStraightModel()
{
}

void DuctStraightModel::addNodesXAxis(tgStructure &s)
{
    s.addNode(0, 0, -m_config.m_ductHeight/2.0);
    s.addNode(m_config.m_distance, 0, -m_config.m_ductHeight/2.0);

    s.addNode(0, 0, m_config.m_ductHeight/2.0);
    s.addNode(m_config.m_distance, 0, m_config.m_ductHeight/2.0);

    s.addNode(0, -m_config.m_ductWidth/2.0, 0);
    s.addNode(m_config.m_distance, -m_config.m_ductWidth/2.0, 0);

    s.addNode(0, m_config.m_ductWidth/2.0, 0);
    s.addNode(m_config.m_distance, m_config.m_ductWidth/2.0, 0);
}

void DuctStraightModel::addNodesYAxis(tgStructure &s)
{
    s.addNode(0, 0, -m_config.m_ductHeight/2.0);
    s.addNode(0, m_config.m_distance, -m_config.m_ductHeight/2.0);

    s.addNode(0, 0, m_config.m_ductHeight/2.0);
    s.addNode(0, m_config.m_distance, m_config.m_ductHeight/2.0);

    s.addNode(-m_config.m_ductWidth/2.0, 0, 0);
    s.addNode(-m_config.m_ductWidth/2.0, m_config.m_distance, 0);

    s.addNode(m_config.m_ductWidth/2.0, 0, 0);
    s.addNode(m_config.m_ductWidth/2.0, m_config.m_distance, 0);
}

void DuctStraightModel::addNodesZAxis(tgStructure &s)
{
    s.addNode(0, -m_config.m_ductHeight/2.0, 0);
    s.addNode(0, -m_config.m_ductHeight/2.0, m_config.m_distance);

    s.addNode(0, m_config.m_ductHeight/2.0, 0);
    s.addNode(0, m_config.m_ductHeight/2.0, m_config.m_distance);

    s.addNode(-m_config.m_ductWidth/2.0, 0, 0);
    s.addNode(-m_config.m_ductWidth/2.0, 0, m_config.m_distance);

    s.addNode(m_config.m_ductWidth/2.0, 0, 0);
    s.addNode(m_config.m_ductWidth/2.0, 0, m_config.m_distance);
}

void DuctStraightModel::addNodes(tgStructure &s)
{
    switch(m_config.m_axis)
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
    const tgBox::Config boxConfig(m_config.m_ductWidth/2.0, m_config.m_wallWidth, 0, m_config.m_friction, 0, 0.01);
    const tgBox::Config boxConfig2(m_config.m_wallWidth, m_config.m_ductHeight/2.0, 0, m_config.m_friction, 0, 0.01);
    
    // Create a structure that will hold the details of this model
    tgStructure s;

    addNodes(s);
    addBoxes(s);

    // Move the structure so it doesn't start in the ground
    if (m_config.m_axis == 0)
    {
        s.move(btVector3(0, m_config.m_ductWidth/2.0+1, 0));
    }
    else if (m_config.m_axis == 2)
    {
        s.move(btVector3(0, m_config.m_ductHeight/2.0+1, 0));
    }

    s.move(m_config.m_startPos);
    
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
