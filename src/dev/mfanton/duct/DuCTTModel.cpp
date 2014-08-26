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
 * @file DuCTTModel.cpp
 * @brief Contains the definition of the members of the class DuCTTModel.
 * $Id$
 */

// This module
#include "DuCTTModel.h"
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
     //
     // see tgBaseString.h for a descripton of some of these rod parameters
     // (specifically, those related to the motor moving the strings.)
     //
     // NOTE that any parameter that depends on units of length will scale
     // with the current gravity scaling. E.g., with gravity as 981,
     // the length units below are in centimeters.
     //
     // Total mass of bars is about 1.5 kg.  Total
     */
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double damping;
        double triangle_length;
        double duct_distance;
        double duct_height;
        double pretension;     
    } c =
   {
       0.00164,     // density (mass / length^3) kg/cm^3 0.00164
       1.27,     // radius (length) 1.27 cm
       10000.0,   // stiffness (mass / sec^2) vectran string
       100.0,     // damping (mass / sec)
       30,     // triangle_length (length) 30 cm
       15,     // duct_distance (distance between tetrahedrons) 15 cm
       22,     // duct_height (length)
       0.05      // Pretension (percentage)
   }
    ;
} // namespace

DuCTTModel::DuCTTModel() :
tgModel() 
{
}

DuCTTModel::~DuCTTModel()
{
    delete m_pStringController;
}

void DuCTTModel::addNodesB(tgStructure& tetra,
                            double edge,
                            double distance,
                            double height)
{
    
    // bottom 0
    tetra.addNode( edge / 2.0, 0, 0); // 0
    // bottom 1
    tetra.addNode( -edge / 2.0, 0, 0); // 1
    // bottom 2
    tetra.addNode(0, height, -edge / 2.0); // 2
    // bottom 3
    tetra.addNode(0, height, edge / 2.0); // 3
    
}


void DuCTTModel::addNodesT(tgStructure& tetra,
                           double edge,
                           double distance,
                           double height)
{
    
    // top 0
    tetra.addNode( edge / 2.0, distance, 0); // 4
    // top 1
    tetra.addNode( -edge / 2.0, distance, 0); // 5
    // top 2
    tetra.addNode(0, distance + height, -edge / 2.0); // 6
    // top 3
    tetra.addNode(0, distance + height, edge / 2.0); // 7
    
}


void DuCTTModel::addRodsB(tgStructure& s)
{
    // bottom tetra
    s.addPair( 0,  1, "rodB");
    s.addPair( 0,  2, "rodB");
    s.addPair( 0,  3, "rodB");
    s.addPair( 1,  2, "rodB");
    s.addPair( 1,  3, "rodB");
    s.addPair( 2,  3, "rodB");

}

void DuCTTModel::addRodsT(tgStructure& s)
{
    
    // top tetra
    s.addPair( 4,  5, "rodT");
    s.addPair( 4,  6, "rodT");
    s.addPair( 4,  7, "rodT");
    s.addPair( 5,  6, "rodT");
    s.addPair( 5,  7, "rodT");
    s.addPair( 6,  7, "rodT");
}

void DuCTTModel::addMuscles(tgStructure& s)
{
    
    s.addPair(0, 4,  "muscle one"); //0
    s.addPair(1, 5,  "muscle two"); //1
    s.addPair(2, 6,  "muscle three"); //2
    s.addPair(3, 7,  "muscle four"); //3
    
    s.addPair(3, 4,  "muscle five"); //4
    s.addPair(2, 4,  "muscle six"); //5
    s.addPair(3, 5,  "muscle seven"); //6
    s.addPair(2, 5,  "muscle eight"); //7

}

void DuCTTModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // rodConfigB has density of 0 so it stays fixed in simulator
    const tgRod::Config rodConfigB(c.radius, 0);
    const tgRod::Config rodConfigT(c.radius, c.density);
    const tgLinearString::Config muscleConfig(c.stiffness, c.damping);
    
    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the bottom tetrahedron
    addNodesB(s, c.triangle_length, c.duct_distance, c.duct_height);
    
    // Add rods to the bottom tetrahedron
    addRodsB(s);
    
    // Add nodes to top tetrahedron
    addNodesT(s, c.triangle_length, c.duct_distance, c.duct_height);
    
    // Add rods to the top tetrahedron
    addRodsT(s);
    
    // Add muscles to the structure
    addMuscles(s);
    
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 10, 0));
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rodB", new tgRodInfo(rodConfigT));
    spec.addBuilder("rodT", new tgRodInfo(rodConfigT));
    spec.addBuilder("muscle", new tgLinearStringInfo(muscleConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgLinearString> (getDescendants());
    
//    // Then attach the pretension controller to each of these muscles to keep
//    // the tensegrity's shape
//    for (std::size_t i = 0; i < allMuscles.size(); i++)
//    {
//        allMuscles[i]->attach(m_pStringController);
//    }
    
    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void DuCTTModel::step(double dt)
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

void DuCTTModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgLinearString*>& DuCTTModel::getAllMuscles() const
{
    return allMuscles;
}
    
void DuCTTModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
