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
 * @file KneeModel.cpp
 * @brief Contains the definition of the members of the class KneeModel.
 * $Id$
 */

// This module
#include "KneeModel.h"
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
        double Knee_height;  
    } c =
   {
       0.2,     // density (mass / length^3)
       0.31,     // radius (length)
       1000.0,   // stiffness (mass / sec^2)
       10.0,     // damping (mass / sec)
       500.0,     // pretension (mass * length / sec^2)
       10.0,     // triangle_length (length)
       10.0,     // triangle_height (length)
       20.0,     // Knee_height (length)
  };
} // namespace

KneeModel::KneeModel() :
tgModel() 
{
}

KneeModel::~KneeModel()
{
}

void KneeModel::addNodes(tgStructure& s,
                            double edge,
                            double width,
                            double height)
{
//tibia and fibia
    // bottom right
    s.addNode(-edge / 2.0, 0, 0); // 0
    // bottom left
    s.addNode( edge / 2.0, 0, 0); // 1
    // bottom front
    s.addNode(0, 0, width); // 2
    // top right
    s.addNode(-edge / 2.0, height, 0); // 3
    // top left
    s.addNode( edge / 2.0, height, 0); // 4
    // top front
    s.addNode(0, height, width); // 5
//humerus
    // bottom right
    s.addNode(-edge / 2.0, height+1, 0); // 6
    // bottom left
    s.addNode( edge / 2.0, height+1, 0); // 7
    // bottom front
    s.addNode(0, height+1, width); // 8
    // top right
    s.addNode(-edge / 2.0, height*2, 0); // 9
    // top left
    s.addNode( edge / 2.0, height*2, 0); // 10
    // top front
    s.addNode(0, height*2, width); // 11

}

void KneeModel::addRods(tgStructure& s)
{
//fibia
    s.addPair( 0,  4, "rod");
    s.addPair( 1,  5, "rod");
    s.addPair( 2,  3, "rod");
//tibia
	s.addPair( 6, 10, "rod");
	s.addPair( 7, 11, "rod");
	s.addPair( 8, 9, "rod");
}

void KneeModel::addMuscles(tgStructure& s)
{
//Tibia and Fibia
    // Bottom Triangle
    s.addPair(0, 1,  "muscle");
    s.addPair(1, 2,  "muscle");
    s.addPair(2, 0,  "muscle");
    
    // Top
    s.addPair(3, 4, "muscle");
    s.addPair(4, 5,  "muscle");
    s.addPair(5, 3,  "muscle");

    //Edges
    s.addPair(0, 3, "muscle");
    s.addPair(1, 4,  "muscle");
    s.addPair(2, 5,  "muscle");
//Joint
	s.addPair(5, 7, "muscle");
	s.addPair(5, 6, "muscle");
	s.addPair(5, 8, "muscle");
	s.addPair(3, 6, "muscle");
	s.addPair(4, 7, "muscle");
//Humerus
    // Bottom Triangle
    s.addPair(6, 7,  "muscle");
    s.addPair(7, 8,  "muscle");
    s.addPair(8, 6,  "muscle");
    
    // Top
    s.addPair(9, 10, "muscle");
    s.addPair(10, 11,  "muscle");
    s.addPair(11, 9,  "muscle");

    //Edges
    s.addPair(6, 9, "muscle");
    s.addPair(7, 10,  "muscle");
    s.addPair(8, 11,  "muscle");
}

void KneeModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config rodConfig(c.radius, c.density);
    const tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension);
    
    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the structure
    addNodes(s, c.triangle_length, c.triangle_height, c.Knee_height);
    
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

void KneeModel::step(double dt)
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

void KneeModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& KneeModel::getAllActuators() const
{
    return allActuators;
}
    
void KneeModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
