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
 * @file NestedBoxTestModel.cpp
 * @brief Contains the implementation of class NestedBoxTestModel
 * @author Brian Tietz
 * $Id$
 */

// This module
#include "NestedBoxTestModel.h"
// This library
#include "core/tgCast.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgBox.h"
#include "core/tgRod.h"
#include "core/tgSphere.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgSphereInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <iostream>
#include <stdexcept>

NestedBoxTestModel::NestedBoxTestModel(size_t segments) : 
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
        tetra.addNode(-edge / 2.0, 0, 0, "heavy");
        // left
        tetra.addNode( edge / 2.0, 0, 0, "heavy");
        // top
        tetra.addNode(0, height, -1.0 * tgUtil::round(std::sqrt(3.0) / 2.0 * height), "light");
        // front
        tetra.addNode(0, height, tgUtil::round(std::sqrt(3.0) / 2.0 * height), "heavy");
    }

    void addPairs(tgStructure& tetra)
    {
        tetra.addPair(0, 1, "back bottom rod");
        tetra.addPair(0, 2, "back right rod");
        tetra.addPair(0, 3, "front right rod");
        tetra.addPair(1, 2, "back left rod");
        tetra.addPair(1, 3, "front left rod");
        tetra.addPair(2, 3, "front top rod");
    }

    void addSegments(tgStructure& snake, const tgStructure& tetra, double edge,
             size_t segmentCount)
    {

        const btVector3 offset(0, 0, -edge * 2);
        for (size_t i = 0; i < segmentCount; ++i)
        {
            tgStructure* const t = new tgStructure(tetra);
            t->addTags(tgString("segment", i + 1));
            t->move((i + 1) * offset);
            // Add a child to the snake
            snake.addChild(t);
        }
    }

    // Add muscles that connect the segments
    void addMuscles(tgStructure& snake)
    {
        const std::vector<tgStructure*> children = snake.getChildren();
            for (size_t i = 1; i < children.size(); ++i)
            {
                tgNodes n0 = children[i-1]->getNodes();
                tgNodes n1 = children[i  ]->getNodes();

                snake.addPair(n0[0], n1[0], "outer right muscle");
                snake.addPair(n0[1], n1[1], "outer left muscle");
                snake.addPair(n0[2], n1[2], "outer top muscle");

                snake.addPair(n0[0], n1[3], "inner right muscle");
                snake.addPair(n0[1], n1[3], "inner left muscle");
                snake.addPair(n0[2], n1[3], "inner top muscle");
            }
    }

    void mapMuscles(NestedBoxTestModel::MuscleMap& muscleMap,
            tgModel& model)
    {
        // Note that tags don't need to match exactly, we could create
        // supersets if we wanted to
        muscleMap["inner left"]  = model.find<tgSpringCableActuator>("inner left muscle");
        muscleMap["inner right"] = model.find<tgSpringCableActuator>("inner right muscle");
        muscleMap["inner top"]   = model.find<tgSpringCableActuator>("inner top muscle");
        muscleMap["outer left"]  = model.find<tgSpringCableActuator>("outer left muscle");
        muscleMap["outer right"] = model.find<tgSpringCableActuator>("outer right muscle");
        muscleMap["outer top"]   = model.find<tgSpringCableActuator>("outer top muscle");
    }

    void trace(const tgStructureInfo& structureInfo, tgModel& model)
    {
        std::cout << "StructureInfo:" << std::endl
          << structureInfo    << std::endl
          << "Model: "        << std::endl
          << model            << std::endl;
    }

} // namespace

void NestedBoxTestModel::setup(tgWorld& world)
{
    const double edge = 30.0;
    const double height = tgUtil::round(std::sqrt(3.0)/2 * edge);
    std::cout << "edge: " << edge << "; height: " << height << std::endl;

    // Create the tetrahedra
    tgStructure tetra;
    addNodes(tetra, edge, height);
    addPairs(tetra);

    // Move the first one so we can create a longer snake.
    // Or you could move the snake at the end, up to you. 
    tetra.move(btVector3(0.0, 2.0, 100.0));

    // Create our snake segments
    tgStructure snake;
    addSegments(snake, tetra, edge, m_segments);
    addMuscles(snake);

    // Create the build spec that uses tags to turn the structure into a real model
    // Note: This needs to be high enough or things fly apart...
    const double density = 4.2 / 3000.0; // kg / length^3 - see app for length
    const double radius = 0.5;
    const double h  = 0.5;
    const tgBox::Config rodConfig(radius, density);
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgBoxInfo(rodConfig));
    
    tgSpringCableActuator::Config muscleConfig(1000, 10);
    //spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
    const tgSphere::Config sphereConfig(0.5, 0.5);
    spec.addBuilder("light", new tgSphereInfo(sphereConfig));
    
    const tgSphere::Config sphereConfig2(0.5, 2.5);
    spec.addBuilder("light", new tgSphereInfo(sphereConfig2));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);
    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. muscles)
    // that we want to control.    
    allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    mapMuscles(muscleMap, *this);

    trace(structureInfo, *this);

    // Actually setup the children
    tgModel::setup(world);
}

void NestedBoxTestModel::step(double dt)
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
    
const std::vector<tgSpringCableActuator*>&
NestedBoxTestModel::getMuscles (const std::string& key) const
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

