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
 * @file FlemonsSpineModelLearningCL.cpp
 * @brief Implementing the cross-linked octahedral complex spine inspired by Tom Flemons
 * @author Brian Tietz
 * @date May 2014
 * @version 1.0.0
 * $Id$
 */

#include "FlemonsSpineModelLearningCL.h"

// This library
#include "core/tgCast.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgBasicContactCableInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"

#include "LinearMath/btVector3.h"
#include <iostream>
#include <algorithm> // std::fill
#include <map>
#include <set>

FlemonsSpineModelLearningCL::FlemonsSpineModelLearningCL(int segments) :   
    BaseSpineModelLearning(segments) 
{
}

FlemonsSpineModelLearningCL::~FlemonsSpineModelLearningCL()
{
}

void FlemonsSpineModelLearningCL::setup(tgWorld& world)
{
    // This is basically a manual setup of a model. There are things that do this for us (@todo: reference the things that do this for us)

    // Rod and Muscle configuration
    
    const double density = 4.2/300.0;  // Note: This needs to be high enough or things fly apart...
    const double radius  = 0.5;
    const double friction = 0.5;
    const double rollFriction = 0.0;
    const double restitution = 0.0;
    const tgRod::Config rodConfig(radius, density, friction, rollFriction, restitution);
    
    const double radius2  = 0.15;
    const double density2 = 1;  // Note: This needs to be high enough or things fly apart...
    const tgRod::Config rodConfig2(radius2, density2);
    
    const double stiffness = 1000.0;
    const double damping = .01*stiffness;
    const double pretension = 0.0;
    
    /// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate
    const tgSpringCableActuator::Config stringConfig(stiffness, damping, pretension, false, 7000, 24);
    
    
    const double passivePretension = 700; // 5 N
    tgSpringCableActuator::Config muscleConfig(2000, 20, passivePretension);
    
    // Calculations for the flemons spine model
    double v_size = 10.0;
    
    // Create the tetrahedra
    tgStructure tetra;

    tetra.addNode(0,0,0);  // center
    tetra.addNode(0.0, v_size, 0.0);   // Top
    tetra.addNode(0.0, -v_size, 0.0);   // Bottom
    tetra.addNode(0.0, 0.0, v_size);   // front
    tetra.addNode(0.0, 0.0, -v_size);   // back
    tetra.addNode(v_size, 0.0, 0.0); // right
    tetra.addNode(-v_size, 0.0, 0.0); // left
    
    tetra.addPair(0,1, "top rod");
    tetra.addPair(0,2, "bottom rod");
    tetra.addPair(0,3, "front rod");
    tetra.addPair(0,4, "back rod");
    tetra.addPair(0,5, "right rod");
    tetra.addPair(0,6, "left rod");
    

    // Create our snake segments
    tgStructure snake;
    const double offsetDist = -v_size *1.25;
    btVector3 offset(0,0,offsetDist); // @todo: there seems to be an issue with Muscle2P connections if the front of a tetra is inside the next one.
    for(std::size_t i = 0; i < m_segments; i++) {
        // @todo: the snake is a temporary variable -- will its destructor be called? If not, where do we delete its children?
        tgStructure* t = new tgStructure(tetra);
        t->addTags(tgString("segment num", i + 1));
        t->move((i + 1)*offset);
        
        if (i % 2 == 1)
        {
            t->addRotation(btVector3(0.0, 0.0, (i + 1) * offsetDist), btVector3(1, 0, 0), M_PI/4.0);
        }
        else
        {
            t->addRotation(btVector3(0.0, 0.0, (i + 1) * offsetDist), btVector3(0, 1, 0), -M_PI/4.0);
        }
        
        //t->addRotation(btVector3(0.0, 0.0, (i + 1) * offsetDist), btVector3(0, 0, 1), M_PI/4.0);
        
        
        snake.addChild(t); // Add a child to the snake
    }
    
    // Move the snake at the end, up to you. 
    snake.addRotation(btVector3(0.0, 0.0, 0.0), btVector3(0, 0, 1), M_PI/4.0);
    snake.move(btVector3(0.0,15.0,100.0));
    //conditionally compile for debugging 
    #if (1)
    // Add muscles that connect the segments
    // Tag the muscles with their segment numbers so CPGs can find
    // them.
    std::vector<tgStructure*> children = snake.getChildren();
    for(std::size_t i = 1; i < children.size(); i++) {
        tgNodes n0 = children[i-1]->getNodes();
        tgNodes n1 = children[i]->getNodes();

        if (i % 2 == 0 )
        {
            
            snake.addPair(n0[2], n1[3], tgString("inner front muscle seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[4], n1[3], tgString("inner right muscle seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[2], n1[5], tgString("inner left muscle seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[4], n1[5], tgString("inner back muscle seg", i-1) + tgString(" seg", i));
            
            #if (1) // Traditional interior crosslink
            snake.addPair(n0[5], n1[3], tgString("inner left muscle2 seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[6], n1[5], tgString("inner back muscle2 seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[2], n1[1], tgString("inner left muscle2 seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[4], n1[2], tgString("inner back muscle2 seg", i-1) + tgString(" seg", i));
     
            #else
            snake.addPair(n0[5], n1[5], tgString("inner left muscle2 seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[6], n1[3], tgString("inner back muscle2 seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[4], n1[1], tgString("inner left muscle2 seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[2], n1[2], tgString("inner back muscle2 seg", i-1) + tgString(" seg", i));
            #endif
            
        }
        else
        {
            
            snake.addPair(n0[6], n1[1], tgString("inner front muscle seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[4], n1[1], tgString("inner right muscle seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[6], n1[3], tgString("inner left muscle seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[4], n1[3], tgString("inner back muscle seg", i-1) + tgString(" seg", i));
            
            #if (1)
            snake.addPair(n0[1], n1[3], tgString("inner left muscle2 seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[2], n1[1], tgString("inner back muscle2 seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[6], n1[5], tgString("inner left muscle2 seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[4], n1[6], tgString("inner back muscle2 seg", i-1) + tgString(" seg", i));

            #else
            snake.addPair(n0[4], n1[5], tgString("inner left muscle2 seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[2], n1[3], tgString("inner back muscle2 seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[1], n1[1], tgString("inner left muscle2 seg", i-1) + tgString(" seg", i));
            snake.addPair(n0[6], n1[6], tgString("inner back muscle2 seg", i-1) + tgString(" seg", i));
            #endif
            
        }
    }
    #endif
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    
    #if (1)
    spec.addBuilder("muscle", new tgBasicContactCableInfo(muscleConfig));
    spec.addBuilder("muscle2", new tgBasicContactCableInfo(stringConfig));
    #else
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("muscle2", new tgBasicActuatorInfo(stringConfig));
    #endif
    
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // Setup vectors for control
    m_allMuscles = find<tgSpringCableActuator> ("muscle2");   
    m_allSegments = this->find<tgModel> ("segment");
    
    #if (0)
    // Debug printing
    std::cout << "StructureInfo:" << std::endl;
    std::cout << structureInfo << std::endl;
    
    std::cout << "Model: " << std::endl;
    std::cout << *this << std::endl;
    
    #endif

    children.clear();
    
    // Actually setup the children
    BaseSpineModelLearning::setup(world);
}

void FlemonsSpineModelLearningCL::teardown()
{
    
    BaseSpineModelLearning::teardown();
      
}

void FlemonsSpineModelLearningCL::step(double dt)
{
   /* CPG update occurs in the controller so that we can decouple it
    * from the physics update
    */
    
    BaseSpineModelLearning::step(dt);  // Step any children
}
