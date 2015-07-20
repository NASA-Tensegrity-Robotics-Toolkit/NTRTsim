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
 * @file RibModel.cpp
 * @brief Implements a spine model with a rib cage
 * @author Brian Tietz
 * @date May 2014
 * @version 1.0.0
 * $Id$
 */

#include "RibModel.h"

#include "core/tgCast.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"

// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>

namespace
{
    void addNodes(tgStructure& tetra, double v_size)
    {
        tetra.addNode(0,0,0);  // center
        tetra.addNode(0.0, v_size, v_size / 2.0);   // top
        tetra.addNode(-v_size, 0.0, -v_size); // left
        tetra.addNode(v_size, 0.0, -v_size); // right
     #if (0) // Attempt at compliant rib attachments
        tetra.addNode(-v_size / 5.0, 0.0, 0.0); // right
        tetra.addNode(v_size / 5.0, 0.0, 0.0); // right
     #endif
        
        //tetra.addNode(v_size, -v_size, -v_size);   // right
        
        //tetra.addNode(-v_size, -v_size, v_size); // left
    }
    
    void ellipseNodes(tgStructure& tetra, double a, double b, double startT, double endT, size_t n) 
    {
        double rodL = (endT - startT) / (double) n;
        
        for(size_t i = 0; i < n; i++)
        { 
            double x = a * cos( startT + i * rodL);
            double y = b * sin( startT + i * rodL);
            // Just build it in xy, can rotate later
            tgNode node( btVector3( x, y, 0.0));
            tetra.addNode( node );
        }
        #if (0)
        // Final Nodes for "feet"
        double x = a * cos( startT + (n - 1) * rodL);
        double y = b * sin( startT + (n - 1) * rodL) - 0.5;
        tgNode node( btVector3( x, y, 0.0));      
        tetra.addNode( node );
        #endif
    }
    
    /**@todo rename to differentiate*/
    void addPairs(tgStructure& tetra)
    {
        tetra.addPair(0,1, "top rod");
        tetra.addPair(0,2, "left rod");
        tetra.addPair(0,3, "right rod");
    #if (0) // extra connection points for muscles
        tetra.addPair(0,4, "leftRib rod");
        tetra.addPair(0,5, "rightRib rod");
    #endif
        //tetra.addPair(0,3, "back rod");
        //tetra.addPair(0,4, "left rod");
    }
    
    /**@todo rename to differentiate*/
    void makePairs(tgStructure& tetra) 
    {   
        size_t n = tetra.getNodes().size();
        std::cout << "Nodes size is " << n << std::endl;
        for(std::size_t i = 1; i < n; i++) {
            tetra.addPair(i-1, i, "rod");
        }
    }
    
    void addSegments(tgStructure& snake, const tgStructure& tetra, 
     const tgStructure& ribs, double edge, size_t segmentCount)
    {
        const btVector3 offset(0, 0, -edge * 1.3);
        const btVector3 rotAxis(0, 1.0, 0);
    for (size_t i = 0; i < segmentCount; ++i)
    {

      tgStructure* const t = new tgStructure(tetra);
      // Need a new pointer for every rib
      tgStructure* const r = new tgStructure(ribs);
      tgStructure* const r2 = new tgStructure(ribs);
      t->addChild(r);
      t->addChild(r2);
      
      tgNodes r01 = r->getNodes();
      tgNodes r02 = r2->getNodes();
      tgNodes t01 = t->getNodes();
      
      size_t r02End = r02.getNodes().size();
      #if (0)
      r2->addRotation(t01[0], rotAxis, M_PI);
      t->addPair(t01[0], r01[0], "multiMuscle connect");
      t->addPair(t01[0], r02[0], "multiMuscle connect");
      #else
      t->addPair(t01[0], r01[0], "rod connect");
      t->addPair(t01[0], r02[0], "rod connect");
      r2->addRotation(r02[0], rotAxis, M_PI);
      #endif
      
      
      t->addTags(tgString("segment num", i + 1));
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
        
        const std::vector<tgStructure*> subChildren1 = children[i-1]->getChildren();
        const std::vector<tgStructure*> subChildren2 = children[i]->getChildren();
        
        tgNodes r10 = subChildren1[0]->getNodes();
        tgNodes r11 = subChildren1[1]->getNodes();
        tgNodes r20 = subChildren2[0]->getNodes();
        tgNodes r21 = subChildren2[1]->getNodes();
        
        snake.addPair(n0[1], n1[1], tgString("outer front muscleAct seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[2], n1[2], tgString("outer right muscleAct seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[3], n1[3], tgString("outer back muscleAct seg", i-1) + tgString(" seg", i));

        snake.addPair(n0[2], n1[1], tgString("inner front muscleAct seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[3], n1[1], tgString("inner right muscleAct seg", i-1) + tgString(" seg", i));
        
        snake.addPair(n0[2], r20[3], tgString("inner front muscleAct seg", i-1) + tgString(" seg", i));
        snake.addPair(n0[3], r21[3], tgString("inner right muscleAct seg", i-1) + tgString(" seg", i));
        
        snake.addPair(r10[5], r20[12], tgString("intercostal muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(r11[5], r21[12], tgString("intercostal muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(r10[12], r20[5], tgString("intercostal muscle seg", i-1) + tgString(" seg", i));
        snake.addPair(r11[12], r21[5], tgString("intercostal muscle seg", i-1) + tgString(" seg", i));
    }
    }

    void trace(const tgStructureInfo& structureInfo, tgModel& model)
    {
        std::cout << "StructureInfo:" << std::endl
          << structureInfo    << std::endl
          << "Model: "        << std::endl
          << model            << std::endl;    
    // Showing the find function
    const std::vector<tgSpringCableActuator*> outerMuscles =
        model.find<tgSpringCableActuator>("outer");
    for (size_t i = 0; i < outerMuscles.size(); ++i)
    {
        const tgSpringCableActuator* const pMuscle = outerMuscles[i];
        assert(pMuscle != NULL);
        std::cout << "Outer muscle: " << *pMuscle << std::endl;
    }
    }

} // namespace

RibModel::RibModel(int segments) :
    BaseSpineModelLearning(segments)
{
}

RibModel::~RibModel()
{
}

void RibModel::setup(tgWorld& world)
{
    double v_size = 3.0;
    
    // Create the spinal processes
    tgStructure vertebrae;
    addNodes(vertebrae, v_size);
    addPairs(vertebrae);

    // Move the first one so we can create a longer snake.
    // Or you could move the snake at the end, up to you. 
    vertebrae.move(btVector3(0.0, 2 * v_size, v_size * m_segments));
    
    // Create ribs and add them to the vertebrae
    double majorAxis  = 6.0;
    double minorAxis  = 4.0;
    double startTheta = M_PI / 2.0;
    double endTheta   = 5.0 * M_PI / 4.0;
    size_t segs       = 15;
    
    tgStructure ribs;
    ellipseNodes(ribs, majorAxis, minorAxis, startTheta, endTheta, segs);
    makePairs(ribs);
    
    #if (0) // Attempt at compliant rib attachments
    ribs.move(btVector3(v_size / 3.0, 2 * v_size - minorAxis, v_size * m_segments));
    #else
    ribs.move(btVector3(0.0, 2 * v_size - minorAxis -.3, v_size * m_segments));
    #endif
    // Create our snake segments
    tgStructure snake;
    addSegments(snake, vertebrae, ribs, v_size, m_segments);
    
    snake.move(btVector3(0.0, majorAxis, 0.0));
    
    addMuscles(snake);

    // Create the build spec that uses tags to turn the structure into a real model
    // Note: This needs to be high enough or things fly apart...
    const double density = 4.2 / 300.0;
    const double radius  = 0.5;
    const double friction = 0.5; // Default is 0.5
    const double rollFriction  = 0.5; // Default is 0.0
    const double restitution  = 0.0; // Default
    
    const tgRod::Config rodConfig(radius, density, friction, rollFriction, restitution);
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));  
    
    const double pretension = 0.0;
    const bool	 hist = false;
    
    tgSpringCableActuator::Config muscleConfig(500, 5, pretension, hist);
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    /// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate
    tgSpringCableActuator::Config muscleConfigAct(1000, 10, pretension, hist, 7000, 24);
    spec.addBuilder("muscleAct", new tgBasicActuatorInfo(muscleConfigAct));
    
    #if (0) // Compliant Rib Attachments
    const double stiffness = 1000;
    const double damping = .01 * stiffness;
    
    tgSpringCableActuator::Config muscleConfig1(stiffness, damping, -M_PI / 2.0);
    tgSpringCableActuator::Config muscleConfig2(stiffness, damping, M_PI / 2.0);
    tgSpringCableActuator::Config muscleConfig3(stiffness, damping, M_PI);
    tgSpringCableActuator::Config muscleConfig4(stiffness, damping, 0);
    
    spec.addBuilder("multiMuscle", new tgBasicActuatorInfo(muscleConfig1));
    spec.addBuilder("multiMuscle", new tgBasicActuatorInfo(muscleConfig2));
    spec.addBuilder("multiMuscle", new tgBasicActuatorInfo(muscleConfig3));
    spec.addBuilder("multiMuscle", new tgBasicActuatorInfo(muscleConfig4));
    #endif
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. muscles)
    // that we want to control.    
    m_allMuscles = find<tgSpringCableActuator> ("muscleAct");
    m_allSegments = find<tgModel> ("segment");

    #if (0)
    trace(structureInfo, *this);
    #endif
   
    // Actually setup the children
    BaseSpineModelLearning::setup(world);
}
void RibModel::teardown()
{
    
    BaseSpineModelLearning::teardown();
      
}

void RibModel::step(double dt)
{
	
	if (dt < 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
    /* CPG update occurs in the controller so that we can decouple it
    * from the physics update
    */
    
		BaseSpineModelLearning::step(dt);  // Step any children
	}
}
