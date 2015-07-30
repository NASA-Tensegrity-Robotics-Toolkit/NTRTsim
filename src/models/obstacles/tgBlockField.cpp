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
 * @file tgBlockField.cpp
 * @brief Contains the implementation of class tgBlockField.
 * $Id$
 */

// This module
#include "tgBlockField.h"
// This library
#include "core/tgBox.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgNode.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
#include <vector>
#include <numeric> // RAND_MAX

tgBlockField::Config::Config(btVector3 origin,
                             btScalar friction, 
                             btScalar restitution,
                             btVector3 minPos, 
                             btVector3 maxPos, 
                             size_t nBlocks, 
                             double blockLength, 
                             double blockWidth, 
                             double blockHeight) :
m_origin(origin),
m_friction(friction),
m_restitution(restitution),
m_minPos(minPos),
m_maxPos(maxPos),
m_nBlocks(nBlocks),
m_length(blockLength),
m_width(blockWidth),
m_height(blockHeight)
{
    assert(m_friction >= 0.0);
    assert(m_restitution >= 0.0);
    assert(m_nBlocks >= 0.0);
    assert(m_length >= 0.0);
    assert(m_width >= 0.0);
    assert(m_height >= 0.0);
}

tgBlockField::tgBlockField() : 
tgModel(),
m_config()
{
    // Seed the random number generator
    /// @todo assess if doing this multiple times in a trial (here and evolution) causes problems
    tgUtil::seedRandom(1);
}

tgBlockField::tgBlockField(tgBlockField::Config& config) :
tgModel(),
m_config(config)
{
    // Seed the random number generator
    /// @todo assess if doing this multiple times in a trial (here and evolution) causes problems
    tgUtil::seedRandom(1);
}

tgBlockField::~tgBlockField() {}
                     
void tgBlockField::setup(tgWorld& world) {
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

void tgBlockField::step(double dt) {
    // Precondition
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    }
    else {
        tgModel::step(dt); // Step any children
    }
}

void tgBlockField::onVisit(tgModelVisitor& r) {
    tgModel::onVisit(r);
}

void tgBlockField::teardown() {
    tgModel::teardown();
} 

// Nodes: center points of opposing faces of rectangles
void tgBlockField::addNodes(tgStructure& s) {
    
    btVector3 fieldSize = m_config.m_maxPos - m_config.m_minPos;
    
    for(size_t i = 0; i < 2 * m_config.m_nBlocks; i += 2) {
        double xOffset = fieldSize.getX() * rand() / RAND_MAX;
        double yOffset = fieldSize.getY() * rand() / RAND_MAX;
        double zOffset = fieldSize.getZ() * rand() / RAND_MAX;
        
        btVector3 offset(xOffset, yOffset, zOffset);
        
        tgNode position = m_config.m_minPos + offset;
        s.addNode(position);
        position += btVector3(0.0, 0.0, m_config.m_length);
        s.addNode(position);
        
        s.addPair(i, i+1, "box");
    }

    s.move(m_config.m_origin); // Set center of field to desired origin position
}

