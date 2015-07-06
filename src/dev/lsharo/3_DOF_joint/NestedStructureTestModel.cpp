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
 * Anonymous namespace for helper functions
 */
namespace
{
    void addNodes(tgStructure& tetra, double radii, double height)
    {
    	double height2 = (height-10.0);
    	double height3 = (height+15.0);

    	double PI = 3.14159;
        // right
        for(int i = 0; i<24;i++){
        tetra.addNode( radii*sin(i*PI/12), radii*cos(i*PI/12), height);
        }
        // front
        tetra.addNode(0, 0, 0);

        tetra.addNode(0, 0, height2);
        tetra.addNode(radii*sin(PI/3.0), radii*cos(PI/3.0), height3);
        tetra.addNode(radii*sin(PI), radii*cos(PI), height3);
        tetra.addNode(radii*sin(5*PI/3.0), radii*cos(5*PI/3.0), height3);


    }

    void addPairs(tgStructure& tetra)
    {
    	for(int i = 0; i<23;i++){
        tetra.addPair(i, i+1, "ring rod");
    	}
        tetra.addPair(0, 23, "ring rod");

        tetra.addPair(0,24, "front top rod");
        tetra.addPair(8, 24, "top rod");
        tetra.addPair(16, 24, "front rod");

        tetra.addPair(25, 26, "test1 rod");
        tetra.addPair(25, 27, "test2 rod");
        tetra.addPair(25, 28, "test3 rod");
        tetra.addPair(26, 27, "test top rod");
        tetra.addPair(27, 28, "test top2 rod");
        tetra.addPair(28, 26, "test top3 rod");
    }

    void addSegments(tgStructure& snake, const tgStructure& tetra, double edge,
             size_t segmentCount)
    {

        const btVector3 offset(0, 0, -edge * 0.6);
        const btVector3 axis(0, 0, 1);
        const btVector3 fixedPoint(20, 20, 20);
        int j = 0;
        for (size_t i = 0; i < segmentCount; ++i)
        {
            tgStructure* const t = new tgStructure(tetra);
            t->addTags(tgString("segment", i + 1));
            t->move((i + 1) * offset);
            t->addRotation( fixedPoint, axis ,j*314159/3);
            // Add a child to the snake
            snake.addChild(t);
        	j++;
        }
    }

    // Add actuators that connect the segments
    void addActuators(tgStructure& snake)
    {
    	const std::vector<tgStructure*> children = snake.getChildren();
    	tgNodes n0 = children[0]->getNodes();
    	snake.addPair(n0[0], n0[25], "test actuator");
    	snake.addPair(n0[8], n0[25], "test2 actuator");
    	snake.addPair(n0[16], n0[25], "test3 actuator");


//            for (size_t i = 1; i < children.size(); ++i)
//            {
//                tgNodes n0 = children[i-1]->getNodes();
//                tgNodes n1 = children[i  ]->getNodes();
//
//                snake.addPair(n0[0], n1[16], "outer right actuator");
//                snake.addPair(n0[8], n1[0], "outer left actuator");
//                snake.addPair(n0[16], n1[8], "outer top actuator");
//                snake.addPair(n0[0], n1[8], "outer2 right actuator");
//                snake.addPair(n0[8], n1[16], "outer2 left actuator");
//                snake.addPair(n0[16], n1[0], "outer2 top actuator");
//
//                snake.addPair(n0[0], n1[24], "inner right actuator");
//                snake.addPair(n0[8], n1[24], "inner left actuator");
//                snake.addPair(n0[16], n1[24], "inner top actuator");
//            }
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
    const double radii = 15.0;
    const double height = 25.0;
    std::cout << "edge: " << radii << "; height: " << height << std::endl;

    // Create the tetrahedra
    tgStructure tetra;
    addNodes(tetra, radii, height);
    addPairs(tetra);

    // Move the first one so we can create a longer snake.
    // Or you could move the snake at the end, up to you. 
    tetra.move(btVector3(20.0, 20.0, 20.0));

    // Create our snake segments
    tgStructure snake;
    addSegments(snake, tetra, radii, m_segments);
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

