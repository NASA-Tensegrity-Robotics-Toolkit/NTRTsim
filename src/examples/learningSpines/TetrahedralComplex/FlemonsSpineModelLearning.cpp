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
 * @file FlemonsSpineModelLearning.cpp
 * @brief Implementing the tetrahedral complex spine inspired by Tom Flemons
 * @author Brian Tietz
 * @date May 2014
 * @version 1.0.0
 * $Id$
 */

// This module
#include "FlemonsSpineModelLearning.h"
// This library
#include "core/tgCast.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgKinematicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <algorithm> // std::fill
#include <iostream>
#include <map>
#include <set>

#define USE_KINEMATIC

FlemonsSpineModelLearning::FlemonsSpineModelLearning(int segments) : 
    BaseSpineModelLearning(segments) 
{
}

FlemonsSpineModelLearning::~FlemonsSpineModelLearning()
{
}

void FlemonsSpineModelLearning::setup(tgWorld& world)
{
    // Rod and Muscle configuration
    // Note: This needs to be high enough or things fly apart...
    const double density = 4.2/300.0;
    const double radius  = 0.5;
    const double friction = 0.5;
    const double rollFriction = 0.0;
    const double restitution = 0.0;
    const tgRod::Config rodConfig(radius, density, friction, rollFriction, restitution);

    const double elasticity = 1000.0;
    const double damping = 10.0;
    const double pretension = 0.0;
    const bool   history = false;
    const double maxTens = 7000.0;
    const double maxSpeed = 12.0;
#ifdef USE_KINEMATIC

    const double mRad = 1.0;
    const double motorFriction = 10.0;
    const double motorInertia = 1.0;
    const bool backDrivable = false;
    tgKinematicActuator::Config motorConfig(elasticity, damping, pretension,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed);

#else
    tgSpringCableActuator::Config muscleConfig(elasticity, damping, pretension, history, maxTens, maxSpeed);
#endif
    // Calculations for the flemons spine model
    double v_size = 10.0;
    
    // Create the tetrahedra
    tgStructure tetra;

    tetra.addNode(0.0, 0.0, 0.0);  // center
    tetra.addNode( v_size,  v_size,  v_size); // front
    tetra.addNode( v_size, -v_size, -v_size); // right
    tetra.addNode(-v_size,  v_size, -v_size); // back
    tetra.addNode(-v_size, -v_size,  v_size); // left
    
    tetra.addPair(0, 1, "front rod");
    tetra.addPair(0, 2, "right rod");
    tetra.addPair(0, 3, "back rod");
    tetra.addPair(0, 4, "left rod");

    // Move the first one so we can create a longer snake.
    // Or you could move the snake at the end, up to you. 
    tetra.move(btVector3(0.0,15.0,100.0));

    // Create our snake segments
    tgStructure snake;
    /// @todo: there seems to be an issue with Muscle2P connections if the front
    /// of a tetra is inside the next one.
    btVector3 offset(0.0, 0.0, -v_size * 1.15);
    for (std::size_t i = 0; i < m_segments; i++)
    {
        /// @todo: the snake is a temporary variable -- 
        /// will its destructor be called?
        /// If not, where do we delete its children?
        tgStructure* const p = new tgStructure(tetra);
        p->addTags(tgString("segment num", i + 1));
        p->move((i + 1.0) * offset);
        snake.addChild(p); // Add a child to the snake
    }
    //conditionally compile for debugging 
#if (1)
    // Add muscles that connect the segments
    // Tag the muscles with their segment numbers so CPGs can find
    // them.
    std::vector<tgStructure*> children = snake.getChildren();
    for (std::size_t i = 1; i < children.size(); i++)
    {
        tgNodes n0 = children[i - 1]->getNodes();
        tgNodes n1 = children[i]->getNodes();
        
        snake.addPair(n0[1], n1[1],
              tgString("outer front muscle seg", i - 1) + tgString(" seg", i));
        snake.addPair(n0[2], n1[2],
              tgString("outer right muscle seg", i - 1) + tgString(" seg", i));
        snake.addPair(n0[3], n1[3],
              tgString("outer back muscle seg",  i - 1) + tgString(" seg", i));
        snake.addPair(n0[4], n1[4],
              tgString("outer top muscle seg",   i - 1) + tgString(" seg", i));
        
        snake.addPair(n0[2], n1[1],
              tgString("inner front muscle seg",  i - 1) + tgString(" seg", i));
        snake.addPair(n0[2], n1[4],
              tgString("inner right muscle seg",  i - 1) + tgString(" seg", i));
        snake.addPair(n0[3], n1[1],
              tgString("inner left muscle seg",   i - 1) + tgString(" seg", i));
        snake.addPair(n0[3], n1[4],
              tgString("inner back muscle seg",   i - 1) + tgString(" seg", i));

    }
#endif
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    
#ifdef USE_KINEMATIC
    spec.addBuilder("muscle", new tgKinematicActuatorInfo(motorConfig));
#else
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
#endif
    
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // Setup vectors for control
    m_allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
     
    m_allSegments = this->find<tgModel> ("segment");
    
#if (0)
    // Debug printing
    std::cout << "StructureInfo:" << std::endl;
    std::cout << structureInfo << std::endl;
    
    std::cout << "Model: " << std::endl;
    std::cout << *this << std::endl;
#endif
    children.clear();
    
    // Actually setup the children, notify controller
    BaseSpineModelLearning::setup(world);
}

void FlemonsSpineModelLearning::teardown()
{
    
    BaseSpineModelLearning::teardown();
      
}

void FlemonsSpineModelLearning::step(double dt)
{
    /* CPG update occurs in the controller so that we can decouple it
    * from the physics update
    */
    
    BaseSpineModelLearning::step(dt);  // Step any children
}
