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

// This module
#include "TetraSpineStaticModel.h"
// This library
#include "core/tgCast.h"
#include "core/tgLinearString.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgLinearStringInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <stdexcept>

/**
 * @file TetraSpineStaticModel.cpp
 * @brief Tetraspine, configured for learning in the NTRT simulator
 * @author Brian Tietz
 * @date May 2014
 * @version 1.0.0
 * $Id$
 */

TetraSpineStaticModel::TetraSpineStaticModel(size_t segments) : 
    BaseSpineModelLearning(segments)
{
}

TetraSpineStaticModel::~TetraSpineStaticModel()
{
}
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
        const btVector3 offset(0, 0, -21.5);
    for (size_t i = 0; i < segmentCount; ++i)
    {
            /// @todo: the snake is a temporary variable -- will its destructor be called?
        /// If not, where do we delete its children?
      tgStructure* const t = new tgStructure(tetra);
	  if (i == 0)
      {
		  t->addTags(tgString("mobile segment num", i + 1));
		  
	  }
	  else
	  {
		  t->addTags(tgString("static segment num", i + 1));
	  }

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

    void mapMuscles(TetraSpineStaticModel::MuscleMap& muscleMap,
            tgModel& model)
    {
        // Note that tags don't need to match exactly, we could create
        // supersets if we wanted to
    muscleMap["inner left"]  = model.find<tgLinearString>("inner left muscle");
    muscleMap["inner right"] = model.find<tgLinearString>("inner right muscle");
    muscleMap["inner top"]   = model.find<tgLinearString>("inner top muscle");
    muscleMap["outer left"]  = model.find<tgLinearString>("outer left muscle");
    muscleMap["outer right"] = model.find<tgLinearString>("outer right muscle");
    muscleMap["outer top"]   = model.find<tgLinearString>("outer top muscle");
    }
	
	void addMarkers(tgStructure& structure, TetraSpineStaticModel& model)
	{
		const std::vector<tgStructure*> children = structure.getChildren();
		tgNodes n0 = children[0]->getNodes();
		
		btRigidBody* firstBody = model.getAllRigids()[0]->getPRigidBody();
		
		abstractMarker marker1(firstBody, n0[3] - firstBody->getCenterOfMassPosition (), btVector3(1, 0, 0), 0);
		
		model.addMarker(marker1);
	}
	
    void trace(const tgStructureInfo& structureInfo, tgModel& model)
    {
        std::cout << "StructureInfo:" << std::endl
          << structureInfo    << std::endl
          << "Model: "        << std::endl
          << model            << std::endl;    
    // Showing the find function
    const std::vector<tgLinearString*> outerMuscles =
        model.find<tgLinearString>("outer");
    for (size_t i = 0; i < outerMuscles.size(); ++i)
    {
        const tgLinearString* const pMuscle = outerMuscles[i];
        assert(pMuscle != NULL);
        std::cout << "Outer muscle: " << *pMuscle << std::endl;
    }
    }

} // namespace

// This is basically a manual setup of a model.
// There are things that do this for us (@todo: reference the things that do this for us)
void TetraSpineStaticModel::setup(tgWorld& world)
{
    const double edge = 38.1;
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
    
#if (0) // Original parameters
    const double density = 4.2 / 300.0;
    const double radius  = 0.5;
    const double friction = 0.5;
    const tgRod::Config rodConfig(radius, density, friction);
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    
    tgLinearString::Config muscleConfig(1000, 100, false, 0, 7000, 24, 10000);
    spec.addBuilder("muscle", new tgLinearStringInfo(muscleConfig));
#else // Params for In Won
    const double density = .00311;
    const double radius  = 0.635;
    const double friction = 0.8;
    const tgRod::Config rodConfig(radius, density, friction);
    tgBuildSpec spec;
    spec.addBuilder("mobile rod", new tgRodInfo(rodConfig));
    
    const tgRod::Config staticConfig(radius, 0.0, friction);
    spec.addBuilder("static rod", new tgRodInfo(staticConfig));
    
    
    tgLinearString::Config muscleConfig(10000, 10, false, 0, 7000, 7.0, 9500);
    spec.addBuilder("muscle", new tgLinearStringInfo(muscleConfig));
#endif
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. muscles)
    // that we want to control.    
    m_allMuscles = this->find<tgLinearString> ("muscle");
    m_allSegments = this->find<tgModel> ("segment");
    mapMuscles(m_muscleMap, *this);
    
    addMarkers(snake, *this);
    
    #if (1)
    trace(structureInfo, *this);
    #endif
    
    // Actually setup the children
    BaseSpineModelLearning::setup(world);
}
      
void TetraSpineStaticModel::teardown()
{

    BaseSpineModelLearning::teardown();
    
} 
    
void TetraSpineStaticModel::step(double dt)
{
    if (dt < 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Step any children, notify observers
        BaseSpineModelLearning::step(dt);
    }
}
