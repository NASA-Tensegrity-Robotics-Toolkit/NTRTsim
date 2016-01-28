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
 * @file SecondKneeModel.cpp
 * @brief Contains the definition of the members of the class SecondKneeModel.
 * $Id$
 */

// This module
#include "TrySecondKneeModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "core/tgSphere.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgSphereInfo.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <stdexcept>
#include <iostream>

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
        double densityA;
        double densityB;
        double radius;
        double stiffness;
        double damping;
        double pretension;
        double triangle_length;
        double triangle_height;
        double Knee_height;
	 double friction;
        double rollFriction;
        double restitution;
    } c =
   {
       9000.0,     // density (mass / length^3)
       0.0,     // density
       0.25,     // radius (length)
       1000.0,   // stiffness (mass / sec^2)
       10.0,     // damping (mass / sec)
       500.0,     // pretension (mass * length / sec^2)
       10.0,     // triangle_length (length)
       10.0,     // triangle_height (length)
       20.0,     // Knee_height (length)
       0.99,      // friction (unitless)
     0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)	
  };
} // namespace

TrySecondKneeModel::TrySecondKneeModel() :
tgModel()
{
}

TrySecondKneeModel::~TrySecondKneeModel()
{
}

void TrySecondKneeModel::addNodes(tgStructure& tetra,
                            double edge,
                            double width,
                            double height)
{

//tibia and fibia (Cross Beams)
    //bottom origin
	tetra.addNode(0,0,0, "sphere");//0
    // bottom right
    tetra.addNode(-edge / 2.0, 0, 0, "sphere"); // 1
    // bottom left
    tetra.addNode( edge / 2.0, 0, 0, "sphere"); // 2
    // bottom front
    tetra.addNode(0, 0, width/1.5, "sphere"); // 3
    // bottom back
    tetra.addNode(0, 0, -width/1.5, "sphere"); //4
    //top origin
	tetra.addNode(0, height, 0, "sphere");//5
    // bottom right
    tetra.addNode(-edge / 2.0, height, 0, "sphere"); // 6
    // bottom left
    tetra.addNode( edge / 2.0, height, 0, "sphere"); // 7
    // bottom front
    tetra.addNode(0, height, width/1.5, "sphere"); // 8
    // bottom back
    tetra.addNode(0, height, -width/1.5, "sphere"); //9


//humerus
	//bottom origin
    tetra.addNode(0, height+2, 0); //10
    // bottom right
    tetra.addNode(-edge/1.5, height+2, 0); // 11
    // bottom left
    tetra.addNode( edge/1.5, height+2, 0); // 12
    // bottom front
    tetra.addNode(0, height+2, width/2); // 13
    // bottom back
    tetra.addNode(0, height+2, -width/2);// 14
	//top origin
    tetra.addNode(0, height*2, 0); //15
    // top right
    tetra.addNode(-edge/1.5, height*2, 0); // 16
    // top left
    tetra.addNode( edge/1.5, height*2, 0); // 17
    // top front
    tetra.addNode(0, height*2, width/2); // 18
    // top back
    tetra.addNode(0, height*2, -width/2);//19

}

void TrySecondKneeModel::addNodesB(tgStructure& tetra,
                            double edge,
                            double width,
                            double height)
{
//tibia and fibia (Cross Beams)
    //bottom origin
	tetra.addNode(0,0,0);//0
    // bottom right
    tetra.addNode(-edge / 2.0, 0, 0); // 1
    // bottom left
    tetra.addNode( edge / 2.0, 0, 0); // 2
    // bottom front
    tetra.addNode(0, 0, width/1.5); // 3
    // bottom back
    tetra.addNode(0, 0, -width/1.5); //4
    //top origin
	tetra.addNode(0, height, 0);//5
    // bottom right
    tetra.addNode(-edge / 2.0, height, 0); // 6
    // bottom left
    tetra.addNode( edge / 2.0, height, 0); // 7
    // bottom front
    tetra.addNode(0, height, width/1.5); // 8
    // bottom back
    tetra.addNode(0, height, -width/1.5); //9

}

void TrySecondKneeModel::addPairs(tgStructure& tetra)
{
//fibia
	//Bottom
    tetra.addPair( 0,  1, "rod");
    tetra.addPair( 0,  2, "rod");
    tetra.addPair( 0,  3, "rod");
    tetra.addPair( 0, 4, "rod");
    tetra.addPair(0, 5, "rod");
	//Top
    tetra.addPair( 5,  6, "rod");
    tetra.addPair( 5,  7, "rod");
    tetra.addPair( 5,  8, "rod");
    tetra.addPair( 5,  9, "rod");


//tibia
	//Bottom
	tetra.addPair( 10, 11, "rod");
	tetra.addPair( 10, 12, "rod");
	tetra.addPair( 10, 13, "rod");
	tetra.addPair( 10, 14, "rod");
	tetra.addPair( 10, 15, "rod");

	//Top
	tetra.addPair(15, 16, "rod");
	tetra.addPair(15, 17, "rod");
	tetra.addPair(15, 18, "rod");
	tetra.addPair(15, 19, "rod");
}

void TrySecondKneeModel::addPairsB(tgStructure& tetra)
{
//fibia
	//Bottom
    tetra.addPair( 0,  1, "rodB");
    tetra.addPair( 0,  2, "rodB");
    tetra.addPair( 0,  3, "rodB");
    tetra.addPair( 0, 4, "rodB");
    tetra.addPair(0, 5, "rodB");
	//Top
    tetra.addPair( 5,  6, "rodB");
    tetra.addPair( 5,  7, "rodB");
    tetra.addPair( 5,  8, "rodB");
    tetra.addPair( 5,  9, "rodB");
}

void TrySecondKneeModel::addMuscles(tgStructure& tetra)
{
//Tibia and Fibia
	tetra.addPair(3, 12, "muscle");//Calve
	tetra.addPair(4, 12, "muscle");
	tetra.addPair(1, 11, "muscle");//Frontal calve
//Joint
	tetra.addPair(8, 13, "muscle");//ACL
	tetra.addPair(9, 14, "muscle");//PCL
	tetra.addPair(6, 16, "muscle");//Patella

//Humerus
	tetra.addPair(8, 17, "muscle");//Hamstring
	tetra.addPair(9, 17, "muscle");
	tetra.addPair(8, 16, "muscle");//Quads
	tetra.addPair(9, 16, "muscle");
}

void TrySecondKneeModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config rodConfigA(c.radius, c.densityA, c.friction, c.rollFriction, c.restitution);
    const tgRod::Config rodConfigB(c.radius, c.densityB, c.friction, c.rollFriction, c.restitution);
    const tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension);
    //welding holders
    const tgSphere::Config weldingConfigA(0.25, c.densityA);
    //fixed segment
    tgStructure tetraB;
    addNodesB(tetraB, c.triangle_length, c.triangle_height, c.Knee_height);
    addPairsB(tetraB);
    tetraB.move(btVector3(0,-20,0));

    // Create a structure that will hold the details of this model
    tgStructure tetra;

    //child
    tgStructure* const tB = new tgStructure(tetraB);
    tetra.addChild(tB);
	
    // Add nodes to the structure
    addNodes(tetra, c.triangle_length, c.triangle_height, c.Knee_height);

    // Add rods to the structure
    addPairs(tetra);

    // Add muscles to the structure
    addMuscles(tetra);

    // Move the structure so it doesn't start in the ground
    tetra.move(btVector3(0, 2, 0));

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfigA));
    spec.addBuilder("rodB", new tgRodInfo(rodConfigB));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("sphere", new tgSphereInfo(weldingConfigA));

    // Create your structureInfo
    tgStructureInfo structureInfo(tetra, spec);

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

void TrySecondKneeModel::step(double dt)
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

void TrySecondKneeModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& TrySecondKneeModel::getAllActuators() const
{
    return allActuators;
}

void TrySecondKneeModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
