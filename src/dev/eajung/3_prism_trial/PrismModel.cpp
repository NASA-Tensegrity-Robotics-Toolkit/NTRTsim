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
 * @file PrismModel.cpp
 * @brief Contains the definition of the members of the class PrismModel.
 * $Id$
 */

// This module
#include "PrismModel.h"
// This library
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
        double pretension;
        double triangle_length;
        double triangle_height;
        double prism_height;  
    } c =
   {
       0.2,     // density (mass / length^3)
       0.31,     // radius (length)
       1000.0,   // stiffness (mass / sec^2)
       10.0,     // damping (mass / sec)
       50.0,     // pretension (mass * length / sec^2)
       10.0,     // triangle_length (length)
       10.0,     // triangle_height (length)
       20.0,     // prism_height (length)
  };
} // namespace

PrismModel::PrismModel() :
tgModel() 
{
}

PrismModel::~PrismModel()
{
}

void PrismModel::addNodes(tgStructure& s,
                            double edge,
                            double width,
                            double height)
{
	//bottom right (index: 0)
	s.addNode(-edge, 0, 0);
	
	//bottom left  (index: 1)
	s.addNode(edge, 0, 0);
	
	//bottom front  (index: 2)
	s.addNode(0, 0, width);

	//bottom back  (index: 3)
	s.addNode(0, 0, -width); 


	//top right (index: 4)
	s.addNode(-edge, height/2.0, 0);

	//top left (index: 5)
	s.addNode(edge, height/2.0, 0);

	//top front (index: 6)
	s.addNode(0, height/2.0, width);

	//top back (index: 7)
	s.addNode(0, height/2.0, -width);

}

void PrismModel::addRods(tgStructure& s)
{
	//bottom square
	/*
    	s.addPair( 0,  3, "rod");
	s.addPair( 0,  2, "rod");
	s.addPair( 1,  2, "rod");
	s.addPair( 1,  3, "rod");
	*/

	//vertical sides
	s.addPair( 0, 4, "rod");
	s.addPair( 1, 5, "rod");
	s.addPair( 2, 6, "rod");
	s.addPair( 3, 7, "rod");

	
}

void PrismModel::addMuscles(tgStructure& s)
{

    	s.addPair( 3,  5, "muscle");
	s.addPair( 0,  7, "muscle");

	s.addPair( 2,  4, "muscle");
	s.addPair( 1,  6, "muscle");	
	

	s.addPair( 6,  7, "muscle");
	s.addPair( 4,  5, "muscle");	


	/*	
	//bottom cables	
	s.addPair( 0,  3, "muscle");
	s.addPair( 0,  2, "muscle");
	s.addPair( 1,  2, "muscle");
	s.addPair( 1,  3, "muscle");

	s.addPair( 2,  3, "muscle");
	s.addPair( 0,  1, "muscle");

	//top cables
	s.addPair( 4,  6, "muscle");
	s.addPair( 4,  7, "muscle");
	s.addPair( 5,  6, "muscle");
	s.addPair( 5,  7, "muscle");
	
	s.addPair( 6,  7, "muscle");
	s.addPair( 4,  5, "muscle");
	
	//side cables
	s.addPair( 1,  6, "muscle");
	s.addPair( 0,  7, "muscle");
	s.addPair( 2,  4, "muscle");
	s.addPair( 1,  7, "muscle");

	s.addPair( 2,  5, "muscle");
	s.addPair( 0,  6, "muscle");
	s.addPair( 3,  4, "muscle");
	s.addPair( 3,  5, "muscle");


	
	//interior cross	
	//s.addPair( 1,  4, "muscle");
	//s.addPair( 3,  6, "muscle");
	*/

	
}

void PrismModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config rodConfig(c.radius, c.density);
    const tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension);
    
    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the structure
    addNodes(s, c.triangle_length, c.triangle_height, c.prism_height);
    
    // Add rods to the structure
    addRods(s);
    
    // Add muscles to the structure
    addMuscles(s);
    
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 10, 0));
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allActuators = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    
    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void PrismModel::step(double dt)
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

void PrismModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& PrismModel::getAllActuators() const
{
    return allActuators;
}
    
void PrismModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
