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
#include "TetraSpineCollisions.h"
// This library
#include "core/tgCast.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgSphere.h"
#include "core/abstractMarker.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgBasicContactCableInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgSphereInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <stdexcept>

/**
 * @file TetraSpineCollisions.cpp
 * @brief Tetraspine, configured for learning in the NTRT simulator
 * @author Brian Tietz
 * @date May 2014
 * @version 1.0.0
 * $Id$
 */

TetraSpineCollisions::TetraSpineCollisions(size_t segments, double scale) : 
    BaseSpineModelLearning(segments),
    scaleFactor(scale / 10.0)
{
}

TetraSpineCollisions::~TetraSpineCollisions()
{
}
namespace
{
    void addNodes(tgStructure& tetra, double edge, double height, double scale)
    {
		// Front segment
        // right
        tetra.addNode(-edge / 2.0, 0, 0, "base");
		// left
		tetra.addNode( edge / 2.0, 0, 0, "base");
        // top
		tetra.addNode(0, height, 0, "base");
		// front
		tetra.addNode(0, edge / 2.0 * tan(M_PI / 6.0), 2.50 * scale, "tip");
		
		// Get the next two nodes from existing nodes:
		tgNodes oldNodes = tetra.getNodes();
		tgNode nn1 = (oldNodes[0] + oldNodes[2])/2.0; // 4 mid
		nn1.addTags("PCB");
		tetra.addNode(nn1);
		tgNode nn2 = (oldNodes[1] + oldNodes[2])/2.0; // 5 mid
		nn2.addTags("PCB");
		tetra.addNode(nn2);
		
		std::cout << (oldNodes[3] - oldNodes[2]).length() << std::endl;
		std::cout << (oldNodes[3] - oldNodes[1]).length() << std::endl;
		std::cout << (oldNodes[3] - oldNodes[0]).length() << std::endl;
		
    }

    void addPairs(tgStructure& tetra)
    {
        tetra.addPair(0, 1, "back bottom rod");
		tetra.addPair(0, 4, "back rightBottom rod");
		tetra.addPair(4, 2, "back rightTop rod");
		tetra.addPair(0, 3, "front right rod");
		tetra.addPair(1, 5, "back leftBottom rod");
		tetra.addPair(5, 2, "back leftTop rod");
		tetra.addPair(1, 3, "front left rod");
		tetra.addPair(2, 3, "front top rod");
    }
	
	
    void addSegments(tgStructure& snake, const tgStructure& tetra, double edge,
             size_t segmentCount)
    {
        const btVector3 offset(0, 0, edge);
		for (size_t i = 0; i < segmentCount; ++i)
		{
				/// @todo: the snake is a temporary variable -- will its destructor be called?
			/// If not, where do we delete its children?
		  tgStructure* const t = new tgStructure(tetra);
		  t->addTags(tgString("segment num", i )); // Use num0, num1, num2 as a tag!!
		  
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

			snake.addPair(n0[0], n1[0], tgString("outer right muscle seg", i));
			snake.addPair(n0[1], n1[1], tgString("outer left muscle seg", i));
			snake.addPair(n0[2], n1[2], tgString("outer top muscle seg", i));

			snake.addPair(n0[0], n1[3], tgString("inner right muscle seg", i));
			snake.addPair(n0[1], n1[3], tgString("inner left muscle seg", i));
			snake.addPair(n0[2], n1[3], tgString("inner top muscle seg", i));
		}
    }

    void mapMuscles(TetraSpineCollisions::MuscleMap& muscleMap,
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
	
	void addMarkers(tgStructure& structure, TetraSpineCollisions& model)
	{
		
		const std::vector<tgStructure*> children = structure.getChildren();
		tgNodes n0 = children[0]->getNodes();
		
		// TODO: consider using the segments vector here
		btRigidBody* firstBody = model.getAllRigids()[0]->getPRigidBody();
		
		std::vector<tgBaseRigid*> myRigids = model.getAllRigids();
#if (0)
		for (int i =0; i < myRigids.size(); i++)
		{
			std::cout << myRigids[i]->mass() << " " <<myRigids[i]->getPRigidBody() << std::endl;
		}
#endif			
		
		abstractMarker marker1(firstBody, n0[3] - firstBody->getCenterOfMassPosition (), btVector3(1, 0, 0), 0);
		
		model.addMarker(marker1);
		
		tgNodes n1 = children[1]->getNodes();
		
		btRigidBody* secondBody = model.getAllRigids()[15]->getPRigidBody();
		
		abstractMarker marker2(secondBody, n1[3] - secondBody->getCenterOfMassPosition (), btVector3(1, 0, 0), 0);
		
		model.addMarker(marker2);
		
		abstractMarker marker3(secondBody, n1[1] - secondBody->getCenterOfMassPosition (), btVector3(1, 0, 0), 0);
		
		model.addMarker(marker3);
		
		abstractMarker marker4(secondBody, n1[0] - secondBody->getCenterOfMassPosition (), btVector3(1, 0, 0), 0);
		
		model.addMarker(marker4);
		
		tgNodes n2 = children[2]->getNodes();
		
		btRigidBody* thirdBody = model.getAllRigids()[29]->getPRigidBody();
		
		abstractMarker marker5(thirdBody, n2[3] - thirdBody->getCenterOfMassPosition (), btVector3(1, 0, 0), 0);
		
		model.addMarker(marker5);
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

// This is basically a manual setup of a model.
// There are things that do this for us (@todo: reference the things that do this for us)
void TetraSpineCollisions::setup(tgWorld& world)
{
    const double edge = 3.8 * scaleFactor;
    const double height = tgUtil::round(std::sqrt(3.0)/2 * edge);
    std::cout << "edge: " << edge << "; height: " << height << std::endl;
	
    // Create the tetrahedra
    tgStructure tetra;
    addNodes(tetra, edge, height, scaleFactor);
    addPairs(tetra);

    // Move the first one so we can create a longer snake.
    // Or you could move the snake at the end, up to you. 
    tetra.move(btVector3(0.0, 8.0, 10.0));

    // Create our snake segments
    tgStructure snake;
    addSegments(snake, tetra, -2.30 * scaleFactor, m_segments);
    addMuscles(snake);

    // Create the build spec that uses tags to turn the structure into a real model
    // Note: This needs to be high enough or things fly apart...
    

    // Params for In Won
    const double radius  = 0.635 * scaleFactor / 10.0;
    const double sphereRadius  = 0.635 * scaleFactor / (10.0);
    const double density = 2.0 *.0201 / (pow(radius, 2) * M_PI * edge); // Mass divided by volume... should there be a way to set this automatically??
    const double friction = 0.5;
    const tgRod::Config rodConfig(radius, density, friction);
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    
    // 1000 is so the units below can be in grams
    const double sphereVolume1 = 1000.0 * 4.0 / 3.0 * M_PI * pow(sphereRadius, 3);
    const double sphereVolume2 = 1000.0 * 4.0 / 3.0 * M_PI * pow(sphereRadius, 3);
    
    const double baseCornerMidD = 180.0 / sphereVolume1;
    const tgSphere::Config baseCornerMidConfig(sphereRadius, baseCornerMidD, friction);
    spec.addBuilder("base", new tgSphereInfo(baseCornerMidConfig));

    const double tipCornerMidD = 120.0 / sphereVolume1;
    const tgSphere::Config tipCornerMidConfig(sphereRadius, tipCornerMidD, friction);
    spec.addBuilder("tip", new tgSphereInfo(tipCornerMidConfig));
    
    const double PCBD = 70.0 / sphereVolume2;
    const tgSphere::Config PCB_1_Config(radius, PCBD, friction);
    spec.addBuilder("PCB", new tgSphereInfo(PCB_1_Config));
    
    
    // Two different string configs
    tgSpringCableActuator::Config muscleConfig(229.16 * 2.0, 20, 0.0, false, 100.0 * scaleFactor, 1.40 * scaleFactor, 0.1, 0.1);
    tgSpringCableActuator::Config muscleConfig2(229.16, 20, 0.0, false, 100.0 * scaleFactor, 1.40 * scaleFactor, 0.1, 0.1);
#if (1)
    spec.addBuilder("top muscle", new tgBasicContactCableInfo(muscleConfig));
    spec.addBuilder("left muscle", new tgBasicContactCableInfo(muscleConfig2));
    spec.addBuilder("right muscle", new tgBasicContactCableInfo(muscleConfig2));
#else
    spec.addBuilder("top muscle", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("left muscle", new tgBasicActuatorInfo(muscleConfig2));
    spec.addBuilder("right muscle", new tgBasicActuatorInfo(muscleConfig2));
#endif
    // Create your structureInfo
    tgStructureInfo structureInfo(snake, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. muscles)
    // that we want to control.    
    m_allMuscles = this->find<tgSpringCableActuator> ("muscle");
    m_allSegments = this->find<tgModel> ("segment");
    mapMuscles(m_muscleMap, *this);
    
    //addMarkers(snake, *this);
    
    #if (0)
    trace(structureInfo, *this);
    #endif
    
    // Actually setup the children
    BaseSpineModelLearning::setup(world);
}
      
void TetraSpineCollisions::teardown()
{

    BaseSpineModelLearning::teardown();
    
} 
    
void TetraSpineCollisions::step(double dt)
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
