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
 * @file VerticalSpineModel.cpp
 * @brief Contains the implementation of class VerticalSpineModel
 * @author Brian Tietz, Drew Sabelhaus, Michael Fanton
 * $Id$
 */

// This module
#include "VerticalSpineModel.h"
// This library
#include "core/tgCast.h"
//#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
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

// @todo move hard-coded parameters into config

VerticalSpineModel::VerticalSpineModel(size_t segments) :
    m_segments(segments),
    tgModel() 
{
}

/**
 * Anonomous namespace for config struct. This makes changing the parameters
 * of the model much easier (they're all at the top of this file!).
 */
namespace
{
    const struct Config
    {
        double densityA;
        double densityB;
        double radius;
        double edge;
        double height;
        double stiffness;
        double damping;  
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        bool   hist;
        double maxTens;
        double targetVelocity;
    } c =
   {
     0.026,    // densityA (kg / length^3)
     0.0,    // densityB (kg / length^3)
     0.5,     // radius (length)
     20.0,      // edge (length)
     tgUtil::round(c.edge / std::sqrt(2.0)),    // height (length)
     1000.0,   // stiffness (kg / sec^2)
     10.0,    // damping (kg / sec)
     0.99,      // friction (unitless)
     0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)
     2452.0,        // pretension
     0,			// History logging (boolean)
     100000,   // maxTens
     10000,    // targetVelocity
      
  };

} // end namespace

// Helper functions, with explicit scopes, moved from implicit namespace.
void VerticalSpineModel::trace(const tgStructureInfo& structureInfo, tgModel& model)
{
    std::cout << "StructureInfo:" << std::endl
    << structureInfo    << std::endl
    << "Model: "        << std::endl
    << model            << std::endl;
}

void VerticalSpineModel::addNodes(tgStructure& tetra, double edge, double height)
{
    // right
    tetra.addNode( c.edge / 2.0, 0, 0); // node 0
    // left
    tetra.addNode( -c.edge / 2.0, 0, 0); // node 1
    // top
    tetra.addNode(0, c.height, -edge / 2.0); // node 2
    // front
    tetra.addNode(0, c.height, edge / 2.0); // node 3
    // middle
    tetra.addNode(0, c.height/2, 0); // node 4

}

void VerticalSpineModel::addPairs(tgStructure& tetra)
{
    tetra.addPair(0, 4, "rod");
    tetra.addPair(1, 4, "rod");
    tetra.addPair(2, 4, "rod");
    tetra.addPair(3, 4, "rod");

}
    
void VerticalSpineModel::addPairsB(tgStructure& tetra)
{
    tetra.addPair(0, 4, "rodB");
    tetra.addPair(1, 4, "rodB");
    tetra.addPair(2, 4, "rodB");
    tetra.addPair(3, 4, "rodB");

}

void VerticalSpineModel::addSegments(tgStructure& snake, const tgStructure& tetra, 
				     double edge, size_t segmentCount)
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
void VerticalSpineModel::addMuscles(tgStructure& snake)
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

void VerticalSpineModel::mapMuscles(VerticalSpineModel::MuscleMap& muscleMap,
            tgModel& model, size_t segmentCount)
{
    // create names for muscles (for getMuscles function)
    
    // vertical muscles
    muscleMap["vertical a"] = model.find<tgSpringCableActuator>("vertical muscle a");
    muscleMap["vertical b"] = model.find<tgSpringCableActuator>("vertical muscle b");
    muscleMap["vertical c"] = model.find<tgSpringCableActuator>("vertical muscle c");
    muscleMap["vertical d"] = model.find<tgSpringCableActuator>("vertical muscle d");
        
    // saddle muscles
    for (size_t i = 1; i < segmentCount ; ++i)
    {
        muscleMap[tgString("saddle", i-1)] = model.find<tgSpringCableActuator>(tgString("saddle muscle seg", i-1));
            
    }
}

/***************************************
 * The primary functions., called from other classes.
 **************************************/
void VerticalSpineModel::setup(tgWorld& world)
{
    // debugging output: edge and height length
    //std::cout << "edge: " << c.edge << "; height: " << c.height << std::endl;
    
    // Create the first fixed snake segment
    // @todo move these hard-coded parameters into config
    tgStructure tetraB;
    addNodes(tetraB, c.edge, c.height);
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
    addNodes(tetra, c.edge, c.height);
    addPairs(tetra);
    
    // Move the first tetrahedra
    // @todo move these hard-coded parameters into config
    tetra.move(btVector3(0.0, -6, 0));
    
    // add rest of segments using original tetra configuration
    addSegments(snake, tetra, c.edge, m_segments);
    
    addMuscles(snake);

    // Create the build spec that uses tags to turn the structure into a real model
    // Note: This needs to be high enough or things fly apart...
    
    // length of inner strut = 12.25 cm
    // m = 1 kg
    // volume of 1 rod = 9.62 cm^3
    // total volume = 38.48 cm^3
    //const double density = 1/38.48; = 0.026 // kg / length^3 - see app for length
    const tgRod::Config rodConfigA(c.radius, c.densityA, c.friction, 
				  c.rollFriction, c.restitution);
    const tgRod::Config rodConfigB(c.radius, c.densityB, c.friction, 
				  c.rollFriction, c.restitution);
    //holder
    const tgRod::Config rodConfigHA(0.1, c.densityA, c.friction,
				    c.rollFriction, c.restitution);
    const tgRod::Config rodConfigHB(0.1, c.densityB, c.friction,
				    c.rollFriction, c.restitution);
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfigA));
    spec.addBuilder("rodB", new tgRodInfo(rodConfigB));


    // set muscle (string) parameters
    // @todo replace acceleration constraint with tgKinematicActuator if needed...
    tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension,
					 c.hist, c.maxTens, c.targetVelocity);
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));

    
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);
    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. muscles)
    // that we want to control.    
    allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    mapMuscles(muscleMap, *this, m_segments);

    //trace(structureInfo, *this);

    // Actually setup the children
    notifySetup();
    tgModel::setup(world);
}

void VerticalSpineModel::step(double dt)
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
VerticalSpineModel::getMuscles (const std::string& key) const
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

const std::vector<tgSpringCableActuator*>& VerticalSpineModel::getAllMuscles() const
{
    return allMuscles;
}

