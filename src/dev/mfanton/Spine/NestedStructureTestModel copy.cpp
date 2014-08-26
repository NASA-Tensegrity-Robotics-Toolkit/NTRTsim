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
 * @file NestedStructureTestModel.cpp
 * @brief Contains the implementation of class NestedStructureTestModel
 * @author Brian Tietz
 * $Id$
 */

// This module
#include "NestedStructureTestModel.h"
// This library
#include "core/tgCast.h"
#include "core/tgLinearString.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgLinearStringInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <iostream>
#include <stdexcept>

NestedStructureTestModel::NestedStructureTestModel(size_t segments) : 
    m_segments(segments),
    tgModel() 
{
}

/**
 * Anonomous namespace for helper functions
 */
namespace
{
    void addNodes(tgStructure& tetra, double edge, double height)
    {
        // right
        tetra.addNode( edge / 2.0, 0, 0); // node 0
        // left
        tetra.addNode( -edge / 2.0, 0, 0); // node 1
        // top
        tetra.addNode(0, -height, -edge / 2.0); // node 2
        // front
        tetra.addNode(0, -height, edge / 2.0); // node 3
        // middle
        tetra.addNode(0, -height/2, 0); // node 4
    }

    void addPairs(tgStructure& tetra)
    {
        tetra.addPair(0, 4, "rod");
        tetra.addPair(1, 4, "rod");
        tetra.addPair(2, 4, "rod");
        tetra.addPair(3, 4, "rod");
    }
    
    void addPairsB(tgStructure& tetra)
    {
        tetra.addPair(0, 4, "rodB");
        tetra.addPair(1, 4, "rodB");
        tetra.addPair(2, 4, "rodB");
        tetra.addPair(3, 4, "rodB");
    }

    
    // Add muscles that connect the segments
    void addMuscles(tgStructure& s)
    {
        s.addPair(0, 4,  "muscle"); //0
        s.addPair(1, 5,  "muscle"); //1
        s.addPair(2, 6,  "muscle"); //2
        s.addPair(3, 7,  "muscle"); //3
        
        s.addPair(5, 3,  "muscle"); //4
        s.addPair(4, 3,  "muscle"); //5
        s.addPair(5, 2,  "muscle"); //6
        s.addPair(4, 2,  "muscle"); //7
    }

    void mapMuscles(NestedStructureTestModel::MuscleMap& muscleMap,
            tgModel& model)
    {
        // Note that tags don't need to match exactly, we could create
        // supersets if we wanted to
        
        muscleMap["string a"] = model.find<tgLinearString>("muscle");
        muscleMap["string b"] = model.find<tgLinearString>("muscle");
        muscleMap["string c"] = model.find<tgLinearString>("muscle");
        muscleMap["string d"] = model.find<tgLinearString>("muscle");
        muscleMap["string e"] = model.find<tgLinearString>("muscle");
        muscleMap["string f"] = model.find<tgLinearString>("muscle");
        muscleMap["string g"] = model.find<tgLinearString>("muscle");
        muscleMap["string h"] = model.find<tgLinearString>("muscle");
    }

    void trace(const tgStructureInfo& structureInfo, tgModel& model)
    {
        std::cout << "StructureInfo:" << std::endl
          << structureInfo    << std::endl
          << "Model: "        << std::endl
          << model            << std::endl;
    }

} // namespace

void NestedStructureTestModel::setup(tgWorld& world)
{
    
    const tgRod::Config rodConfigB(c.radius, 0);
    const tgRod::Config rodConfigT(c.radius, c.density);
    const tgLinearString::Config muscleConfig(c.stiffness, c.damping);
    
    
    const double edge = 30.0;
    //const double height = tgUtil::round(edge / std::sqrt(2.0));
    const double height = 22;
    std::cout << "edge: " << edge << "; height: " << height << std::endl;

    // Create the first tetrahedra
    tgStructure tetra;
    addNodes(tetra, edge, height);
    addPairs(tetra);

    // Move the first tetrahedra
    tetra.move(btVector3(0.0, 15.0, 0));

    // Create our snake segments
    tgStructure snake;
    addSegments(snake, tetra, edge, m_segments);
    addMuscles(snake);

    // Create the build spec that uses tags to turn the structure into a real model
    // Note: This needs to be high enough or things fly apart...
    const double density = 4.2 / 300.0; // kg / length^3 - see app for length
    const double radius  = 0.5;
    const tgRod::Config rodConfig(radius, density);
    const tgRod::Config rodConfigB(radius, 0);
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("rodB", new tgRodInfo(rodConfigB));
    
    tgLinearString::Config muscleConfig(1000, 10);
    spec.addBuilder("muscle", new tgLinearStringInfo(muscleConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);
    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. muscles)
    // that we want to control.    
    allMuscles = tgCast::filter<tgModel, tgLinearString> (getDescendants());
    mapMuscles(muscleMap, *this);

    trace(structureInfo, *this);

    // Actually setup the children
    tgModel::setup(world);
}

void NestedStructureTestModel::step(double dt)
{
    if (dt < 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        // Step any children
        tgModel::step(dt);
    }
}
    
const std::vector<tgLinearString*>&
NestedStructureTestModel::getMuscles (const std::string& key) const
{
    const MuscleMap::const_iterator it = muscleMap.find(key);
    if (it == muscleMap.end())
    {
        throw std::invalid_argument("Key '" + key + "' not found in muscle map");
    }
    else
    {
        return it->second;
    }
}

