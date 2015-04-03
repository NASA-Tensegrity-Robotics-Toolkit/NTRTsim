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
#include "core/tgBasicActuator.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
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
    tgModel(),
    m_segments(segments)
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
        tetra.addNode(-edge / 2.0, 0, 0);
        // left
        tetra.addNode( edge / 2.0, 0, 0);
        // top
        tetra.addNode(0, height, 0);
        // front
        tetra.addNode(0, height / 2.0, tgUtil::round(std::sqrt(3.0) / 2.0 * height));
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

        const btVector3 offset(0, 0, -edge * 0.6);
        for (size_t i = 0; i < segmentCount; ++i)
        {
            tgStructure* const t = new tgStructure(tetra);
            t->addTags(tgString("segment", i + 1));
            t->move((i + 1) * offset);
            // Add a child to the snake
            snake.addChild(t);
        }
    }

    // Add actuators that connect the segments
    void addActuators(tgStructure& snake)
    {
        const std::vector<tgStructure*> children = snake.getChildren();
            for (size_t i = 1; i < children.size(); ++i)
            {
                tgNodes n0 = children[i-1]->getNodes();
                tgNodes n1 = children[i  ]->getNodes();

                snake.addPair(n0[0], n1[0], "outer right actuator");
                snake.addPair(n0[1], n1[1], "outer left actuator");
                snake.addPair(n0[2], n1[2], "outer top actuator");

                snake.addPair(n0[0], n1[3], "inner right actuator");
                snake.addPair(n0[1], n1[3], "inner left actuator");
                snake.addPair(n0[2], n1[3], "inner top actuator");
            }
    }

    void mapActuators(NestedStructureTestModel::ActuatorMap& actuatorMap,
            tgModel& model)
    {
        // Note that tags don't need to match exactly, we could create
        // supersets if we wanted to
        actuatorMap["inner left"]  = model.find<tgBasicActuator>("inner left actuator");
        actuatorMap["inner right"] = model.find<tgBasicActuator>("inner right actuator");
        actuatorMap["inner top"]   = model.find<tgBasicActuator>("inner top actuator");
        actuatorMap["outer left"]  = model.find<tgBasicActuator>("outer left actuator");
        actuatorMap["outer right"] = model.find<tgBasicActuator>("outer right actuator");
        actuatorMap["outer top"]   = model.find<tgBasicActuator>("outer top actuator");
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
    addActuators(snake);

    // Create the build spec that uses tags to turn the structure into a real model
    // Note: This needs to be high enough or things fly apart...
    const double density = .00311; // kg / length^3 - see app for length
    const double radius  = 0.5;
    const tgRod::Config rodConfig(radius, density);
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    
    tgBasicActuator::Config actuatorConfig(1000, 10);
    spec.addBuilder("actuator", new tgBasicActuatorInfo(actuatorConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);
    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. actuators)
    // that we want to control.    
    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
    mapActuators(actuatorMap, *this);

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
    
const std::vector<tgBasicActuator*>&
NestedStructureTestModel::getActuators (const std::string& key) const
{
    const ActuatorMap::const_iterator it = actuatorMap.find(key);
    if (it == actuatorMap.end())
    {
        throw std::invalid_argument("Key '" + key + "' not found in actuator map");
    }
    else
    {
        return it->second;
    }
}

