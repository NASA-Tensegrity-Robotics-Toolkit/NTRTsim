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
 * @file HingeModel.cpp
 * @brief Contains the definition of the members of the class HingeModel.
 * $Id$
 */

// This module
#include "HingeModel.h"
// This library
#include "controllers/PretensionController.h"
#include "core/tgLinearString.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgLinearStringInfo.h"
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
       0.05,     // density (mass / length^3)
       0.31,     // radius (length)
       1000.0,   // stiffness (mass / sec^2)
       10.0,     // damping (mass / sec)
       10.0,     // triangle_length (length)
       10.0,     // triangle_height (length)
       20.0,     // prism_height (length)
       0.05      // Pretension (percentage)
  };
} // namespace



HingeModel::HingeModel() :
tgModel() 
{
}

HingeModel::~HingeModel()
{
    delete m_pStringController;
}

void HingeModel::addNodes(tgStructure& tetra,
                            double edge,
                            double width,
                            double height)
{

    tetra.addNode(0, 0, 0);
    tetra.addNode(10, 0, 0);
    tetra.addNode(0, 0, 10);
}

void HingeModel::addRods(tgStructure& s)
{
    s.addPair( 0,  1, "t rod");
    s.addPair( 0,  2, "b rod");
}

void HingeModel::addMuscles(tgStructure& s)
{
}


void HingeModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    const tgRod::Config rodConfig(c.radius, c.density);
    const tgLinearString::Config muscleConfig(c.stiffness, c.damping);
    
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
    spec.addBuilder("muscle", new tgLinearStringInfo(muscleConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgLinearString> (getDescendants());
    
    // Then attach the pretension controller to each of these muscles to keep
    // the tensegrity's shape
   
    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void HingeModel::step(double dt)
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

void HingeModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgLinearString*>& HingeModel::getAllMuscles() const
{
    return allMuscles;
}
    
void HingeModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
