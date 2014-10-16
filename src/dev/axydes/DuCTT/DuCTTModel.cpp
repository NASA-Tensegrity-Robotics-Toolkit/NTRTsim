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
#include "core/tgSphere.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgRodHingeInfo.h"
#include "tgcreator/tgLinearStringInfo.h"
#include "tgPrismaticInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgSphereInfo.h"
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
        //tetra prams
        double triangle_length;
        double duct_distance;
        double duct_height;
        //rod params
        double density;
        double prismRadius;
        double prismExtent;
        double vertRodRadius;
        double innerRodRadius;
        //sphere tip params
        double tipRad;
        double tipDens;
        double tipFric;
        //string params
        double stiffness;
        double damping;
        double pretension;
        double maxVertStringVel;
        double maxSaddleStringVel;
        double maxStringForce;
    } c =
   {
       30,     // triangle_length (length) 30 cm
       15,     // duct_distance (distance between tetrahedrons) 15 cm
       22,    // duct_height (length)
       0.00164,     // density (mass / length^3) kg/cm^3 0.00164
       1.524, // prismatic joint radius 1.524 cm
       10.16, // prismatic joint max extension 10.16 cm
       1.27, // vertical rod radius 1.27 cm
       2.0955, // inner rod radius 2.0955 cm
       1.524, // prismatic joint tip radius 1.524 cm
       1, // prismatic joint tip density (mas / length^3) kg/cm^3
       1, // prismatic joint tip friction
       10000.0,   // stiffness (mass / sec^2) vectran string
       100.0,     // damping (mass / sec)
       0.05,     // Pretension (percentage)
       25.4, // max velocity of vertical string motors (cm/s) 25.4cm/s
       8.5, // max velocity of saddle string motors (cm/s) 8.5cm/s
       50 // max force to exert on all strings (Newtons) 50 N
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

    if (distance == 0)
    {
        tetra.addNode(bottomRight.x(), bottomRight.y(), bottomRight.z(), "sphere"); // 0
        tetra.addNode(bottomLeft.x(), bottomLeft.y(), bottomLeft.z(), "sphere"); // 1
        tetra.addNode(topBack); // 2
        tetra.addNode(topFront); // 3
    }
    else
    {
        tetra.addNode(bottomRight); // 0
        tetra.addNode(bottomLeft); // 1

        tetra.addNode(topBack); // 2
        tetra.addNode(topFront); // 3

    //commented out until i can figure out why they cause AABB overflow in Bullet
//        tetra.addNode(topBack.x(), topBack.y(), topBack.z(), "sphere"); // 2
//        tetra.addNode(topFront.x(), topFront.y(), topFront.z(), "sphere"); // 3
    }


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

void DuCTTModel::addRods(tgStructure& s, int startNode)
{
    // for one tetra
    //right rods
    s.addPair( startNode+8, startNode+12, "vert rod");
    s.addPair( startNode+10, startNode+14, "vert rod");

    //left rods
    s.addPair( startNode+9, startNode+13, "vert rod");
    s.addPair( startNode+11, startNode+15, "vert rod");

    if (startNode == 0)
    {
        //bottom tetra
        // bottom rods
        s.addPair( startNode+0, startNode+4, "prism rod");
        s.addPair( startNode+5, startNode+1, "prism rod");

        //top rods
        s.addPair( startNode+2, startNode+3, "inner rod");

	    s.addPair( startNode+4, startNode+5, "prismatic");
    }
    else
    {
        //top tetra
        // bottom rods
        s.addPair( startNode+0, startNode+1, "inner rod");

        //top rods
//        s.addPair( startNode+2, startNode+6, "prism rod");
        s.addPair( startNode+2, startNode+6, "static rodT");
        s.addPair( startNode+7, startNode+3, "prism rod");

	    s.addPair( startNode+6, startNode+7, "prismatic");
    }

    //bottom right hinges
    s.addPair( startNode+0, startNode+8, "hinge");
    s.addPair( startNode+0, startNode+10, "hinge");

    //bottom left hinges
    s.addPair( startNode+1, startNode+9, "hinge2");
    s.addPair( startNode+1, startNode+11, "hinge2");

    //top front hinges
    s.addPair( startNode+3, startNode+12, "hinge");
    s.addPair( startNode+3, startNode+13, "hinge");

    //top back hinges
    s.addPair( startNode+2, startNode+14, "hinge2");
    s.addPair( startNode+2, startNode+15, "hinge2");
}

void DuCTTModel::addMuscles(tgStructure& s, int topNodesStart)
{
    //vertical strings
    s.addPair(0, topNodesStart+0,  "vert string one"); //0
    s.addPair(1, topNodesStart+1,  "vert string two"); //1
    s.addPair(2, topNodesStart+2,  "vert string three"); //2
    s.addPair(3, topNodesStart+3,  "vert string four"); //3
    
    //saddle strings
    s.addPair(3, topNodesStart+0,  "saddle string five"); //4
    s.addPair(2, topNodesStart+0,  "saddle string six"); //5
    s.addPair(3, topNodesStart+1,  "saddle string seven"); //6
    s.addPair(2, topNodesStart+1,  "saddle string eight"); //7
}

void DuCTTModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // rodConfigB has density of 0 so it stays fixed in simulator
    const tgRod::Config prismRodConfig(c.prismRadius, c.density);
    const tgRod::Config prismRodConfigT(c.prismRadius, 0);
    const tgRod::Config vertRodConfig(c.vertRodRadius, c.density);
    const tgRod::Config innerRodConfig(c.innerRodRadius, c.density);

    const tgLinearString::Config vertStringConfig(c.stiffness, c.damping, false, 0, c.maxStringForce, c.maxVertStringVel);
    const tgLinearString::Config saddleStringConfig(c.stiffness, c.damping, false, 0, c.maxStringForce, c.maxSaddleStringVel);

    const tgPrismatic::Config prismConfig(c.prismExtent);
    const tgSphere::Config sphereConfig(c.tipRad, c.tipDens, c.tipFric);

    const tgRodHinge::Config hingeConfig(-SIMD_PI, SIMD_PI, 0);
    const tgRodHinge::Config hingeConfig2(-SIMD_PI, SIMD_PI,1);
    
    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the bottom tetrahedron
    addNodes(s, c.triangle_length, 0, c.duct_height);
    
    // Add rods to the bottom tetrahedron
    addRods(s);
    
    // Add nodes to top tetrahedron
    addNodes(s, c.triangle_length, c.duct_distance, c.duct_height);
    
    // Add rods to the top tetrahedron
    addRods(s, 16);
    
    // Add muscles to the structure
    addMuscles(s, 16);
    
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 10, 0));
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("prism rod", new tgRodInfo(prismRodConfig));
    spec.addBuilder("static rodT", new tgRodInfo(prismRodConfigT));
    spec.addBuilder("vert rod", new tgRodInfo(vertRodConfig));
    spec.addBuilder("inner rod", new tgRodInfo(innerRodConfig));

    spec.addBuilder("vert string", new tgLinearStringInfo(vertStringConfig));
    spec.addBuilder("saddle string", new tgLinearStringInfo(saddleStringConfig));

    spec.addBuilder("prismatic", new tgPrismaticInfo(prismConfig));
    spec.addBuilder("sphere", new tgSphereInfo(sphereConfig));

    spec.addBuilder("hinge", new tgRodHingeInfo(hingeConfig));
    spec.addBuilder("hinge2", new tgRodHingeInfo(hingeConfig2));

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
