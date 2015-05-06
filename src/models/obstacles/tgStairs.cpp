/*
 * Copyright © 2014, United States Government, as represented by the
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
 * @file tgStairs.cpp
 * @brief Contains the implementation of class tgStairs.
 * $Id$
 */

// This module
#include "tgStairs.h"
// This library
#include "core/tgBox.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgNode.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
#include <vector>
#include <numeric> // RAND_MAX

tgStairs::Config::Config(btVector3 origin,
                             btScalar friction, 
                             btScalar restitution, 
                             size_t nBlocks, 
                             double stairWidth, 
                             double stepWidth, 
                             double stepHeight,
                             double angle) :
m_origin(origin),
m_friction(friction),
m_restitution(restitution),
m_nBlocks(nBlocks),
m_length(stairWidth),
m_width(stepWidth),
m_height(stepHeight),
m_angle(angle)
{
    assert(m_friction >= 0.0);
    assert(m_restitution >= 0.0);
    assert(m_nBlocks >= 0.0);
    assert(m_length >= 0.0);
    assert(m_width >= 0.0);
    assert(m_height >= 0.0);
}

tgStairs::tgStairs() : 
tgModel(),
m_config()
{
}

tgStairs::tgStairs(tgStairs::Config& config) :
tgModel(),
m_config(config)
{
}

tgStairs::~tgStairs() {}
                     
void tgStairs::setup(tgWorld& world) {
    // Density and roll friction are set to zero (respectively)
    const tgBox::Config boxConfig(m_config.m_width / 2.0, m_config.m_height / 2.0, 0.0, m_config.m_friction, 0.0, m_config.m_restitution);

    // Start creating the structure
    tgStructure s;
    addNodes(s);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("box", new tgBoxInfo(boxConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // Actually setup the children
    tgModel::setup(world);
}

void tgStairs::step(double dt) {
    // Precondition
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    }
    else {
        tgModel::step(dt); // Step any children
    }
}

void tgStairs::onVisit(tgModelVisitor& r) {
    tgModel::onVisit(r);
}

void tgStairs::teardown() {
    tgModel::teardown();
} 

// Nodes: center points of opposing faces of rectangles
void tgStairs::addNodes(tgStructure& s) {
    
    tgNode position1(0.0, 0.0, 0.0);
    tgNode position2(0.0, 0.0, m_config.m_length);
    btVector3 offset(m_config.m_width, m_config.m_height, 0.0);
    
    for(size_t i = 0; i < 2 * m_config.m_nBlocks; i += 2) {
        s.addNode(position1);
        s.addNode(position2);
        
        s.addPair(i, i+1, "box");
        
        position1 += offset;
        position2 += offset;
    }
    
    btVector3 point(0.0, 0.0, 0.0);
    btVector3 axis(0.0, 1.0, 0.0);
    s.addRotation(point, axis, m_config.m_angle);
    
    s.move(m_config.m_origin); // Set center of field to desired origin position
}

