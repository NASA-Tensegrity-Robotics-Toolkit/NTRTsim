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
 * @file SpineTestModel.cpp
 * @brief Contains the implementation of class SpineTestModel
 * @author Brian Tietz
 * $Id$
 */

// This module
#include "SpineTestModel.h"
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

SpineTestModel::SpineTestModel(size_t segments) :
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
        tetra.addNode(0, height, -edge / 2.0); // node 2
        // front
        tetra.addNode(0, height, edge / 2.0); // node 3
        // middle
        tetra.addNode(0, height/2, 0); // node 4
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

    void addSegments(tgStructure& snake, const tgStructure& tetra, double edge,
             size_t segmentCount)
    {

        //const btVector3 offset(0, 0, -edge * 1.15);
        const btVector3 offset(0, 7.5, 0);
        for (size_t i = 1; i < segmentCount; ++i)
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
                
                // vertical muscles
                snake.addPair(n0[0], n1[0], "vertical muscle a");
                snake.addPair(n0[1], n1[1], "vertical muscle b");
                snake.addPair(n0[2], n1[2], "vertical muscle c");
                snake.addPair(n0[3], n1[3], "vertical muscle d");

                // saddle muscles
                snake.addPair(n0[2], n1[1], tgString("saddle muscle seg", i-1));
                snake.addPair(n0[3], n1[1], tgString("saddle muscle seg", i-1));
                snake.addPair(n0[2], n1[0], tgString("saddle muscle seg", i-1));
                snake.addPair(n0[3], n1[0], tgString("saddle muscle seg", i-1));
            }
    }

    void mapMuscles(SpineTestModel::MuscleMap& muscleMap,
            tgModel& model, size_t segmentCount)
    {
        // create names for muscles (for getMuscles function)
    
        // vertical muscles
        muscleMap["vertical a"] = model.find<tgLinearString>("vertical muscle a");
        muscleMap["vertical b"] = model.find<tgLinearString>("vertical muscle b");
        muscleMap["vertical c"] = model.find<tgLinearString>("vertical muscle c");
        muscleMap["vertical d"] = model.find<tgLinearString>("vertical muscle d");
        
        // saddle muscles
        for (size_t i = 1; i < segmentCount ; ++i)
        {
            muscleMap[tgString("saddle", i-1)] = model.find<tgLinearString>(tgString("saddle muscle seg", i-1));
            
        }
    }

    void trace(const tgStructureInfo& structureInfo, tgModel& model)
    {
        std::cout << "StructureInfo:" << std::endl
          << structureInfo    << std::endl
          << "Model: "        << std::endl
          << model            << std::endl;
    }

} // namespace

void SpineTestModel::setup(tgWorld& world)
{
    const double edge = 20.0;
    const double height = tgUtil::round(edge / std::sqrt(2.0));
    //const double height = 22;
    std::cout << "edge: " << edge << "; height: " << height << std::endl;
    
    // Create the first fixed snake segment
    tgStructure tetraB;
    addNodes(tetraB, edge, height);
    addPairsB(tetraB);
    tetraB.move(btVector3(0.0, 2, 0));
    
    // Create our snake segments
    tgStructure snake;
    
    // add 1st child to snake
    tgStructure* const tB = new tgStructure(tetraB);
    snake.addChild(tB);
    tB->addTags(tgString("segment", 1));
    
    // Create the first non-fixed tetrahedra
    tgStructure tetra;
    addNodes(tetra, edge, height);
    addPairs(tetra);
    
    // Move the first tetrahedra
     tetra.move(btVector3(0.0, -6, 0));
    
    // add rest of segments using original tetra configuration
    addSegments(snake, tetra, edge, m_segments);
    
    addMuscles(snake);

    // Create the build spec that uses tags to turn the structure into a real model
    // Note: This needs to be high enough or things fly apart...
    
    // length of inner strut = 12.25 cm
    // m = 1 kg
    // volume of 1 rod = 9.62 cm^3
    // total volume = 38.48 cm^3
    const double density = 1 / 38.48; // kg / length^3 - see app for length
    const double radius  = 0.5;
    const tgRod::Config rodConfig(radius, density);
    const tgRod::Config rodConfigB(radius, 0);
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("rodB", new tgRodInfo(rodConfigB));
    
    // set muscle (string) parameters
    const double stiffness = 1000 ;
    const double damping = 10;
    tgLinearString::Config muscleConfig(stiffness, damping);
    spec.addBuilder("muscle", new tgLinearStringInfo(muscleConfig));

    
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);
    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. muscles)
    // that we want to control.    
    allMuscles = tgCast::filter<tgModel, tgLinearString> (getDescendants());
    mapMuscles(muscleMap, *this, m_segments);

    trace(structureInfo, *this);

    // Actually setup the children
    tgModel::setup(world);
}

void SpineTestModel::step(double dt)
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
SpineTestModel::getMuscles (const std::string& key) const
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

const std::vector<tgLinearString*>& SpineTestModel::getAllMuscles() const
{
    return allMuscles;
}

