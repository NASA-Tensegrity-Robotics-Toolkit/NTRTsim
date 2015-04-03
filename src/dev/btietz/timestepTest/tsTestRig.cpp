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
 * @file tsTestRig.cpp
 * @brief Contains the definition of the members of the class tsTestRig.
 * $Id$
 */

// This module
#include "tsTestRig.h"
// This library
#include "core/tgKinematicActuator.h"
#include "tgcreator/tgKinematicActuatorInfo.h"

#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>

/**
 * Anonomous namespace so we don't have to declare the config in
 * the header.
 */
namespace
{
    /**
     * Configuration parameters so they're easily accessable.
     * All parameters must be positive.
     */
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double damping;
        double triangle_length;
        double triangle_height;
        double prism_height;
        double pretension;     
    } c =
   {
       2,     // density (mass / length^3)
       0.31,     // radius (length)
       1000.0,   // stiffness (mass / sec^2)
       10.0,     // damping (mass / sec)
       10.0,     // triangle_length (length)
       10.0,     // triangle_height (length)
       20.0,     // prism_height (length)
       0.05      // Pretension (percentage)
  };
} // namespace

tsTestRig::tsTestRig(bool kinematic) :
tgModel(),
useKinematic(kinematic)
{
}

tsTestRig::~tsTestRig()
{

}

void tsTestRig::addNodes(tgStructure& s)
{
    s.addNode(0.0, -1.0, 0.0); // 0
    s.addNode(0.0, 0.0, 0.0); // 1
    s.addNode( 0.0, 10.0, 0.0); // 2
    s.addNode(0.0, 11.0, 0.0); // 3

}

void tsTestRig::addRods(tgStructure& s)
{
    s.addPair( 0,  1, "rod");
    s.addPair( 2,  3, "rod2");
}

void tsTestRig::addMuscles(tgStructure& s)
{

    s.addPair(1, 2,  "muscle");

}

void tsTestRig::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    const tgRod::Config rodConfig(c.radius, c.density);
    const tgRod::Config rodConfig2(c.radius, 0.0);
    // String config needs to be inside boolean switch
        
    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the structure
    addNodes(s);
    
    // Add rods to the structure
    addRods(s);
    
    // Add muscles to the structure
    addMuscles(s);
    
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 5, 0));
    
    // Create the build spec that uses tags to turn the structure into a real model
    // The top rod will be massless - fixed in space
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("rod2", new tgRodInfo(rodConfig2));
    
    if (useKinematic)
    {
		const tgKinematicActuator::Config muscleConfig(c.stiffness, c.damping);
		spec.addBuilder("muscle", new tgKinematicActuatorInfo(muscleConfig));
	}
	else
	{
		const tgBasicActuator::Config muscleConfig(c.stiffness, c.damping);
		spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
	}
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    
    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
    
    totalTime = 0.0;
    reached = false;
}

void tsTestRig::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        totalTime += dt;
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        
        if (useKinematic)
        {
            allMuscles[0]->setControlInput(-580.0);
        }
        else
        {
            allMuscles[0]->setControlInput(5.0, dt);
        }
        if (allMuscles[0]->getRestLength() <= 5.0 && !reached)
        {
			std::cout << "Rest length below 5.0 at: " << totalTime << std::endl;
			reached = true;
		}
        
        tgModel::step(dt);  // Step any children
		//std::cout << allMuscles[0]->getRestLength() << std::endl;
    }
    
}

void tsTestRig::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& tsTestRig::getAllMuscles() const
{
    return allMuscles;
}
    
void tsTestRig::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
