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
#include "tgHingeInfo.h"
#include "tgcreator/tgLinearStringInfo.h"
#include "tgPrismaticInfo.h"
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
//m_pStringController(new PretensionController(c.pretension)),
tgModel()
{
}

DuCTTModel::~DuCTTModel()
{
    delete m_pStringController;
}

void DuCTTModel::addNodes(tgStructure& tetra,
                            double edge,
                            double distance,
                            double height,
                            double distBtHinges,
                            double distBtNodes)
{
    //Main nodes
    btVector3 bottomRight = btVector3(edge/2.0, distance, 0);
    btVector3 bottomLeft = btVector3(-edge/2.0, distance, 0);

    btVector3 topFront = btVector3(0, distance+height, edge/2.0);
    btVector3 topBack = btVector3(0, distance+height, -edge/2.0);

    //Prismatic nodes
    btVector3 bottomMidRight = btVector3(0.01, distance, 0);
    btVector3 bottomMidLeft = btVector3(-0.01, distance, 0);

    btVector3 topMidFront = btVector3(0, distance+height, 0.01);
    btVector3 topMidBack = btVector3(0, distance+height, -0.01);

    std::cout << bottomRight << " " << bottomLeft << " " << topFront << " " << topBack << std::endl;

    //bottom Hinge nodes
    btVector3 RightFrontSlope = topFront - bottomRight;
    btVector3 LeftFrontSlope = topFront - bottomLeft;
    btVector3 RightBackSlope = topBack - bottomRight;
    btVector3 LeftBackSlope = topBack - bottomLeft;

    btVector3 bottomRightFront = bottomRight + distBtHinges*RightFrontSlope;
    btVector3 bottomLeftFront = bottomLeft + distBtHinges*LeftFrontSlope;
    btVector3 bottomRightBack = bottomRight + distBtHinges*RightBackSlope;
    btVector3 bottomLeftBack = bottomLeft + distBtHinges*LeftBackSlope;
    bottomRight = bottomRight - btVector3(distBtNodes,0,0);
    bottomLeft = bottomLeft + btVector3(distBtNodes,0,0);

    //top Hinge nodes
    btVector3 topRightFront = topFront - distBtHinges*RightFrontSlope;
    btVector3 topLeftFront = topFront - distBtHinges*LeftFrontSlope;
    btVector3 topRightBack = topBack - distBtHinges*RightBackSlope;
    btVector3 topLeftBack = topBack - distBtHinges*LeftBackSlope;
    topFront = topFront - btVector3(0,0,distBtNodes);
    topBack = topBack + btVector3(0,0,distBtNodes);

    std::cout << bottomRight << " " << bottomRightFront << " " << bottomRightBack << std::endl;
    std::cout << bottomLeft << " " << bottomLeftFront << " " << bottomLeftBack << std::endl;
    std::cout << topFront << " " << topRightFront << " " << topRightBack << std::endl;
    std::cout << topBack << " " << topLeftFront << " " << topLeftBack << std::endl;
    std::cout << std::endl;

    tetra.addNode(bottomRight); // 0
    tetra.addNode(bottomLeft); // 1

    tetra.addNode(topBack); // 2
    tetra.addNode(topFront); // 3

    tetra.addNode(bottomMidRight); // 4
    tetra.addNode(bottomMidLeft); // 5

    tetra.addNode(topMidBack); // 6
    tetra.addNode(topMidFront); // 7

    tetra.addNode(bottomRightFront); // 8
    tetra.addNode(bottomLeftFront); // 9

    tetra.addNode(bottomRightBack); // 10
    tetra.addNode(bottomLeftBack); // 11

    tetra.addNode(topRightFront); // 12
    tetra.addNode(topLeftFront); // 13

    tetra.addNode(topRightBack); // 14
    tetra.addNode(topLeftBack); // 15
}

void DuCTTModel::addRodsB(tgStructure& s)
{
    // bottom tetra
    s.addPair( 0,  4, "rodT");
    s.addPair( 5,  1, "rodB");

    s.addPair( 8,  12, "rodB");
    s.addPair( 10,  14, "rodB");

    s.addPair( 9,  13, "rodB");
    s.addPair(11,  15, "rodB");

    s.addPair( 2,  6, "rodT");
    s.addPair( 7,  3, "rodB");

    s.addPair( 4,  5, "prismatic");
    s.addPair( 6,  7, "prismatic");

    //bottom right hinges
    s.addPair( 0,  8, "hinge");
    s.addPair( 0,  10, "hinge");

    //bottom left hinges
    s.addPair( 1,  9, "hinge");
    s.addPair( 1,  11, "hinge");

    //top front hinges
    s.addPair( 3,  12, "hinge");
    s.addPair( 3,  13, "hinge");

    //top back hinges
    s.addPair( 2,  14, "hinge");
    s.addPair( 2,  15, "hinge");
}

void DuCTTModel::addRodsT(tgStructure& s)
{
    // top tetra
    s.addPair( 8,  12, "rodT");
    s.addPair( 13,  9, "rodT");

    s.addPair( 8,  10, "rodT");
    s.addPair( 8,  11, "rodT");

    s.addPair( 9,  10, "rodT");
    s.addPair( 9,  11, "rodT");

    s.addPair( 10,  14, "rodT");
    s.addPair( 15,  11, "rodT");

    s.addPair( 12,  13, "prismatic");
    s.addPair( 14, 15, "prismatic");
}

void DuCTTModel::addMuscles(tgStructure& s)
{
    s.addPair(0, 8,  "muscle one"); //0
    s.addPair(1, 9,  "muscle two"); //1
    s.addPair(2, 10,  "muscle three"); //2
    s.addPair(3, 11,  "muscle four"); //3
    
    s.addPair(3, 8,  "muscle five"); //4
    s.addPair(2, 8,  "muscle six"); //5
    s.addPair(3, 9,  "muscle seven"); //6
    s.addPair(2, 9,  "muscle eight"); //7
}

void DuCTTModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // rodConfigB has density of 0 so it stays fixed in simulator
    const tgRod::Config rodConfigB(c.radius, c.density);
    const tgRod::Config rodConfigT(c.radius, 0);
    const tgLinearString::Config muscleConfig(c.stiffness, c.damping);
    const tgPrismatic::Config prismConfig(.1);
//    const tgHinge::Config hingeConfig(-SIMD_PI, SIMD_PI);
    const tgHinge::Config hingeConfig(-0.15,0);
    
    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the bottom tetrahedron
    addNodes(s, c.triangle_length, 0, c.duct_height);
    
    // Add rods to the bottom tetrahedron
    addRodsB(s);
    
    // Add nodes to top tetrahedron
//    addNodes(s, c.triangle_length, c.duct_distance, c.duct_height);
    
    // Add rods to the top tetrahedron
//    addRodsT(s);
    
    // Add muscles to the structure
//    addMuscles(s);
    
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 10, 0));
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rodB", new tgRodInfo(rodConfigB));
    spec.addBuilder("rodT", new tgRodInfo(rodConfigT));
    spec.addBuilder("muscle", new tgLinearStringInfo(muscleConfig));
    spec.addBuilder("prismatic", new tgPrismaticInfo(prismConfig));
    spec.addBuilder("hinge", new tgHingeInfo(hingeConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgLinearString> (getDescendants());
    allPrisms = tgCast::filter<tgModel, tgPrismatic> (getDescendants());

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
