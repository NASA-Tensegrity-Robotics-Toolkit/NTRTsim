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
 * @file TwoRodsModel.cpp
 * @brief Contains the implementation of class TwoRodsModel.
 * @author Drew Sabelhaus
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// This module
#include "TwoRodsModel.h"
// This library
#include "core/tgCompressionSpringActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgCompressionSpringActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>

namespace
{
    // using tgCompressionSpringActuator here.
    // frictional parameters are for the tgRod objects.
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
        double springRestLength;
    } c =
   {
     0.688,    // density (kg / length^3)
     0.31,     // radius (length)
     80.0,   // stiffness (kg / sec^2) was 1500
     20.0,    // damping (kg / sec)
     16.84,     // rod_length (length)
     7.5,      // rod_space (length)
     0.99,      // friction (unitless)
     0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)
     5.0,   // springRestLength (length)
  };
} // namespace

// Constructor: does nothing. All the good stuff happens when the 'Info' classes
// are passed to the tgBuilders and whatnot.
TwoRodsModel::TwoRodsModel() : tgModel() 
{
}

// Destructor: does nothing
TwoRodsModel::~TwoRodsModel()
{
}

// a helper function to add a bunch of nodes
void TwoRodsModel::addNodes(tgStructure& s)
{
    const double half_length = c.rod_length / 2;

    //    s.addNode(-c.rod_space,  -half_length, 0);            // 0
    //s.addNode(-c.rod_space,   half_length, 0);            // 1
    //s.addNode( c.rod_space,  -half_length, 0);            // 2
    //s.addNode( c.rod_space,   half_length, 0);            // 3
    s.addNode(0,            c.rod_space,   -half_length); // 6
    s.addNode(0,            c.rod_space,    half_length); // 7
    s.addNode(-half_length, 0,            c.rod_space);   // 8
    s.addNode( half_length, 0,            c.rod_space);   // 9
    s.addNode(-half_length, 0,           -c.rod_space);   // 10
    s.addNode( half_length, 0,           -c.rod_space);   // 11
}

// helper function to tag two sets of nodes as rods
void TwoRodsModel::addRods(tgStructure& s)
{
    s.addPair( 0,  1, "rod");
    s.addPair( 2,  3, "rod");
}

// helper function to add our single compression spring actuator
void TwoRodsModel::addActuators(tgStructure& s)
{
    s.addPair(0, 2,  "compressionSpring");
}

// Finally, create the model!
void TwoRodsModel::setup(tgWorld& world)
{
    // config struct for the rods
    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);

    // config struct for the compression spring
    tgCompressionSpringActuator::Config compressionSpringConfig(c.stiffness, c.damping,  
					    c.springRestLength);
            
    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addActuators(s);
    s.move(btVector3(0, 10, 0));

    // Add a rotation. This is needed if the ground slopes too much,
    // otherwise  glitches put a rod below the ground.
    btVector3 rotationPoint = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis = btVector3(0, 1, 0);  // y-axis
    double rotationAngle = M_PI/2;
    s.addRotation(rotationPoint, rotationAxis, rotationAngle);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("compressionSpring", new tgCompressionSpringActuatorInfo(compressionSpringConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allActuators = tgCast::filter<tgModel, tgCompressionSpringActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void TwoRodsModel::step(double dt)
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

void TwoRodsModel::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgCompressionSpringActuator*>& TwoRodsModel::getAllActuators() const
{
    return allActuators;
}
    
void TwoRodsModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
