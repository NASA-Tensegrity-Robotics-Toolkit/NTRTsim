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
 * @file CaterpillarModel.cpp
 * @brief Contains the implementation of class CaterpillarModel.
 * $Id$
 */

// This module
#include "CaterpillarModel.h"
// This library
#include "core/tgSpringCableActuator.h"
#include "core/tgRod.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>

namespace
{
    // see tgBaseString.h for a descripton of some of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    // Note: This current model of the SUPERball rod is 1.5m long by 3 cm radius,
    // which is 0.00424 m^3.
    // For SUPERball v1.5, mass = 3.5kg per strut, which comes out to 
    // 0.825 kg / (decimeter^3).

    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double damping;
        double rod_length;
        double rod_space;    
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        bool   hist;
        double maxTens;
        double targetVelocity;
    } c =
   {
     0.688,    // density (kg / length^3)
     0.31,     // radius (length)
     613.0,   // stiffness (kg / sec^2) was 1500
     200.0,    // damping (kg / sec)
     16.84,     // rod_length (length)
     7.5,      // rod_space (length)
     0.99,      // friction (unitless)
     0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)
     919.5,        // pretension
     0,			// History logging (boolean)
     100000,   // maxTens
     10000,    // targetVelocity

     // Use the below values for earlier versions of simulation.
     // 1.006,    
     // 0.31,     
     // 300000.0, 
     // 3000.0,   
     // 15.0,     
     // 7.5,      
  };
  
  void addSegments(tgStructure& snake, const tgStructure& tetra, double edge,
             size_t segmentCount)
    {

        const btVector3 offset(-edge, 0, 0.0);
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
    void addSegmentMuscles(tgStructure& snake)
    {
        const std::vector<tgStructure*> children = snake.getChildren();
        
		for (size_t i = 1; i < children.size(); ++i)
		{
			    tgNodes n0 = children[i-1]->getNodes();
                tgNodes n1 = children[i  ]->getNodes();
                
                if (i % 2 == 0)
                {
					snake.addPair(n0[4], n1[5], "segment rod");
				}
				else
				{
					snake.addPair(n0[6], n1[7], "segment rod");
				}	
		}
	}
    
      // Add muscles that connect the segments
    void addSegmentMusclesOld(tgStructure& snake)
    {
        const std::vector<tgStructure*> children = snake.getChildren();
            for (size_t i = 1; i < children.size(); ++i)
            {
                tgNodes n0 = children[i-1]->getNodes();
                tgNodes n1 = children[i  ]->getNodes();
				
				tgNode mid0((n0[0] + n1[0])/2.0);
				tgNode mid1((n0[1] + n1[1])/2.0);
				tgNode mid2((n0[2] + n1[2])/2.0);
				tgNode mid3((n0[3] + n1[3])/2.0);
#if (0)				
				if (i == 1)
				{
					btScalar segDist = (n0[0] - n1[0]).length() / 2.0;
					tgNode start0 = n0[0] + btVector3(segDist, 0.0, 0.0);
					tgNode start1 = n0[1] + btVector3(segDist, 0.0, 0.0);
					tgNode start2 = n0[2] + btVector3(segDist, 0.0, 0.0);
					tgNode start3 = n0[3] + btVector3(segDist, 0.0, 0.0);
					
					snake.addPair(start0, start1, "segment rod");
					snake.addPair(start2, start3, "segment rod");
					
					snake.addPair(n0[5], start0, "segment muscle front one");
					snake.addPair(n0[5], start2, "segment muscle front two");
					snake.addPair(n0[7], start1, "segment muscle front three");
					snake.addPair(n0[7], start3, "segment muscle front four");
					
					snake.addPair(n0[8], start0, "segment muscle rear one");
					snake.addPair(n0[8], start1, "segment muscle rear two");
					snake.addPair(n0[9], start2, "segment muscle rear three");
					snake.addPair(n0[9], start3, "segment muscle rear four");    
					
				}
				if (i == children.size() - 1)
				{
					btScalar segDist = (n0[0] - n1[0]).length() / 2.0;
					tgNode end0 = n1[0] - btVector3(segDist, 0.0, 0.0);
					tgNode end1 = n1[1] - btVector3(segDist, 0.0, 0.0);
					tgNode end2 = n1[2] - btVector3(segDist, 0.0, 0.0);
					tgNode end3 = n1[3] - btVector3(segDist, 0.0, 0.0);
					
					snake.addPair(end0, end1, "segment rod");
					snake.addPair(end2, end3, "segment rod");
					
					snake.addPair(n1[4], end0, "segment muscle rear one");
					snake.addPair(n1[4], end2, "segment muscle rear two");
					snake.addPair(n1[6], end1, "segment muscle rear three");
					snake.addPair(n1[6], end3, "segment muscle rear four");
					
					snake.addPair(n1[10], end0, "segment muscle rear one");
					snake.addPair(n1[10], end1, "segment muscle rear two");
					snake.addPair(n1[11], end2, "segment muscle rear three");
					snake.addPair(n1[11], end3, "segment muscle rear four");
				}
#endif		
				snake.addPair(mid0, mid1, "segment rod");
				snake.addPair(mid2, mid3, "segment rod");
				
                snake.addPair(n0[4], mid0, "segment muscle rear one");
                snake.addPair(n0[4], mid2, "segment muscle rear two");
                snake.addPair(n0[6], mid1, "segment muscle rear three");
                snake.addPair(n0[6], mid3, "segment muscle rear four");

                snake.addPair(n1[5], mid0, "segment muscle front one");
                snake.addPair(n1[5], mid2, "segment muscle front two");
                snake.addPair(n1[7], mid1, "segment muscle front three");
                snake.addPair(n1[7], mid3, "segment muscle front four");
        
                snake.addPair(n0[10], mid0, "segment muscle rear one");
                snake.addPair(n0[10], mid1, "segment muscle rear two");
                snake.addPair(n0[11], mid2, "segment muscle rear three");
                snake.addPair(n0[11], mid3, "segment muscle rear four");  
                
                snake.addPair(n1[8], mid0, "segment muscle rear one");
                snake.addPair(n1[8], mid1, "segment muscle rear two");
                snake.addPair(n1[9], mid2, "segment muscle rear three");
                snake.addPair(n1[9], mid3, "segment muscle rear four");         


            }
    }  
} // namespace

CaterpillarModel::CaterpillarModel() : tgModel() 
{
}

CaterpillarModel::~CaterpillarModel()
{
}

void CaterpillarModel::addNodes(tgStructure& s)
{
    const double half_length = c.rod_length / 2;

    s.addNode(-c.rod_space,  -half_length, 0);            // 0
    s.addNode(-c.rod_space,   half_length, 0);            // 1
    s.addNode( c.rod_space,  -half_length, 0);            // 2
    s.addNode( c.rod_space,   half_length, 0);            // 3
    s.addNode(0,           -c.rod_space,   -half_length); // 4
    s.addNode(0,           -c.rod_space,    half_length); // 5
    s.addNode(0,            c.rod_space,   -half_length); // 6
    s.addNode(0,            c.rod_space,    half_length); // 7
    s.addNode(-half_length, 0,            c.rod_space);   // 8
    s.addNode( half_length, 0,            c.rod_space);   // 9
    s.addNode(-half_length, 0,           -c.rod_space);   // 10
    s.addNode( half_length, 0,           -c.rod_space);   // 11
}

void CaterpillarModel::addRods(tgStructure& s)
{
    s.addPair( 0,  1, "y rod");
    s.addPair( 2,  3, "y rod");
    s.addPair( 4,  5, "z rod");
    s.addPair( 6,  7, "z rod");
    s.addPair( 8,  9, "x rod");
    s.addPair(10, 11, "x rod");
}

void CaterpillarModel::addMuscles(tgStructure& s)
{
    s.addPair(0, 4,  "muscle");
    s.addPair(0, 5,  "muscle");
    s.addPair(0, 8,  "muscle");
    s.addPair(0, 10, "muscle");

    s.addPair(1, 6,  "muscle");
    s.addPair(1, 7,  "muscle");
    s.addPair(1, 8,  "muscle");
    s.addPair(1, 10, "muscle");

    s.addPair(2, 4,  "muscle");
    s.addPair(2, 5,  "muscle");
    s.addPair(2, 9,  "muscle");
    s.addPair(2, 11, "muscle");

    s.addPair(3, 7,  "muscle");
    s.addPair(3, 6,  "muscle");
    s.addPair(3, 9,  "muscle");
    s.addPair(3, 11, "muscle");

    s.addPair(4, 2,  "muscle");
    s.addPair(4, 10, "muscle");
    s.addPair(4, 11, "muscle");

    s.addPair(5, 8,  "muscle");
    s.addPair(5, 9,  "muscle");

    s.addPair(6, 10, "muscle");
    s.addPair(6, 11, "muscle");

    s.addPair(7, 8,  "muscle");
    s.addPair(7, 9,  "muscle");

}

void CaterpillarModel::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);

    tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension, c.hist, 
					    c.maxTens, c.targetVelocity);
            
    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addMuscles(s);
    
    btVector3 rotationPoint = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis = btVector3(0, 1, 0);  // y-axis
    double rotationAngle = M_PI/2;
    s.addRotation(rotationPoint, rotationAxis, rotationAngle);
    
    s.move(btVector3(0, 10, 0));
	
	tgStructure caterpillar;
	addSegments(caterpillar, s, 2.0 * c.rod_length, 5);
	addSegmentMuscles(caterpillar);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(caterpillar, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void CaterpillarModel::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void CaterpillarModel::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& CaterpillarModel::getAllMuscles() const
{
    return allMuscles;
}
    
void CaterpillarModel::teardown()
{
    tgModel::teardown();
}
