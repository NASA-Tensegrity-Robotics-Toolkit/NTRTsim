Skip to content
Sign up Sign in
This repository  
Explore
Features
Enterprise
Pricing
 Watch 28  Star 46  Fork 16 NASA-Tensegrity-Robotics-Toolkit/NTRTsim
Branch: DuCTT NTRTsim/src/dev/axydes/Hinge/HingeModel.cpp
Alexander Xydes on Feb 26 DuCTT: Renaming tgTouchSensorSphereModel to tgTouchSensorModel.
1 contributor
RawBlameHistory    Executable File  222 lines (184 sloc)  5.9 KB
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
#include "../DuCTT/robot/tgTouchSensorSphereInfo.h"
#include "../DuCTT/robot/tgTouchSensorModel.h"
// This library
#include "core/abstractMarker.h"
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
}

void HingeModel::addNodes(tgStructure& tetra,
                            double edge,
                            double width,
                            double height)
{

}

void HingeModel::addRods(tgStructure& s)
{
}

void HingeModel::addMuscles(tgStructure& s)
{
}


void HingeModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    const tgRod::Config rodConfig2(c.radius, 0);
    const tgRod::Config rodConfig(c.radius, c.density);
//    const tgHinge::Config hingeConfig(-SIMD_PI, SIMD_PI);
    const tgRodHinge::Config hingeConfig(0, 0.1745329);
    const tgSphere::Config sphereConfig(1, 0.1, 0.5);

    // Create a structure that will hold the details of this model
    tgStructure s;
    tgStructure sGhost;
    
//    s.addNode(9.5, 0, 0);
//    s.addNode(0, 0, 0);

//    s.addNode(9.5, 10, 0);
//    s.addNode(0, 1, 0);

//    s.addNode(1, 9.5, 0);
//    s.addNode(0, 0, 0);

//    s.addNode(4, 9.5, 10);
//    s.addNode(0, 1, 0);

    s.addNode(0, 10, 0, "sphere");
    sGhost.addNode(0, 10, 0, "sphere");

    //bottom
    s.addNode(-15, 0, 0);
    sGhost.addNode(-15, 0, 0);
    s.addNode(15, 0, 0);

    //top
    s.addNode(0, 22, -15);
    s.addNode(0, 22, 15);

    s.addNode(14.6, 0, 0);
//    s.addNode(0, 22, 14.99);

    s.addPair( 0,  1, "t rod");
//    s.addPair( 2,  3, "t rod");
    s.addPair( 4,  3, "b rod");

    s.addPair( 1,  4, "hinge");
    
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 5, 0));
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("sphere", new tgSphereInfo(sphereConfig));
//    spec.addBuilder("b rod", new tgRodInfo(rodConfig));
    spec.addBuilder("t rod", new tgRodInfo(rodConfig2));
//    spec.addBuilder("hinge", new tgRodHingeInfo(hingeConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    tgBuildSpec specGhost;
    specGhost.addBuilder("sphere", new tgTouchSensorSphereInfo(sphereConfig));
    specGhost.addBuilder("t rod", new tgRodInfo(rodConfig2));
    tgStructureInfo structureInfoGhost(sGhost, specGhost);
    structureInfoGhost.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    std::vector<tgSphere*> spheres = find<tgSphere>("sphere");

    std::vector<tgTouchSensorModel*> allTouchSensors = find<tgTouchSensorModel>("sphere");
    for (size_t i=0; i<allTouchSensors.size(); i++)
    {
        abstractMarker marker (spheres[i]->getPRigidBody(), btVector3(0,0,0), btVector3(255,0,0), 0);
        allTouchSensors[i]->addMarker(marker);
    }

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

const std::vector<tgSpringCableActuator*>& HingeModel::getAllMuscles() const
{
    return allMuscles;
}
    
void HingeModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
Status API Training Shop Blog About Pricing
© 2015 GitHub, Inc. Terms Privacy Security Contact Help
