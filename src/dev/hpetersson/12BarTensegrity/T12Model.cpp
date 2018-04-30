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
 * @file T12Model.cpp
z * @brief Contains the implementation of class T12Model.
 * $Id$
 */

// This module
#include "T12Model.h"
#include "T12Controller.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgBasicContactCableInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>

using namespace std;

namespace
{
    // Note: This model of the 12 bar rod will be 1.0m long by 3cm (0.03m) radius,
    // which is 1.00m*pi*(0.030)^2 = 0.002827m^3.
   
    // The density and mass will be heavily inspired by SUPERball v1.5, which had mass = 3.5kg per strut and length 1.5m,
    // giving the density [kg/m^3] comes out to:
    // density = 3.45kg/0.008545132m^3 = 403.738642402kg/m^3 = 0.403738642402 kg/dm^3

    // Using the same density (0.403738642402 kg/dm^3, gives the mass of one of our rods
    // mass = 403.738642402 * 0.002827 = 1.141544118 kg

    // see tgBaseString.h for a descripton of some of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1
    // Thus,
    // Rod Length = 17.00
    // Rod Radius = 0.40
    // density = 0.40374 (length is cubed, so decimal moves 3 places)

    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        double density;
        double radius;
        double stiffnessA;
        double stiffnessB;
        double damping;
        double rod_length;
        double rod_space;    
        double friction;
        double rollFriction;
        double restitution;
        double pretensionA; 
        double pretensionB;
        bool   hist;
        double maxTens;
        double targetVelocity;
    } c =
   {
     2.7,      // density (kg / length^3)
     0.0475,      // radius (length)
     998.25,    // stiffnessA (kg / sec^2), generally less stiff than B.
     1000,   // stiffnessB (kg / sec^2)
     50.0,      // damping (kg / sec)
     4.5,     // rod_length (length)
     4.5/4,   // rod_space (length)
     0.99,      // friction (unitless)
     0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)
     399.0,        // pretensionA (force), from (0.4)*(998)
     1000.0,        // pretensionB (force), from (0.4)*(3152) 1261.0
     0,			// History logging (boolean)
     100000,   // maxTens
     10000    // targetVelocity

     // Use the below values for earlier versions of simulation.
     // 1.152,    // density (kg / length^3)
     //	0.45,     // radius (length)
     //	613.0,    // stiffness_passive (kg / sec^2)
     //	2854.5,   // stiffness_active (kg / sec^2)
  };
} // namespace

T12Model::T12Model() : tgModel() 
{
}

T12Model::~T12Model()
{
}

void T12Model::addNodes(tgStructure& s)
{
    //const double half_length = c.rod_length / 2;

    s.addNode( 0,	0,	0);     // 0
    s.addNode( 0,	4.5,	0);	// 1
    s.addNode( 2.32,	0,	1.38);  // 2
    s.addNode( 2.32,	4.5,	1.38);  // 3
    s.addNode( 2.32,	0,     -1.38);  // 4
    s.addNode( 2.32,	4.5,   -1.38);  // 5
    s.addNode(-0.62,	1.8,   -1.81);  // 6
    s.addNode( 3.49,	0.35,	0);     // 7
    s.addNode(-1.24,	3.1,	0.25);  // 8
    s.addNode( 2.72,	1.4,	2.38);  // 9
    s.addNode( 0,	4.15,  -1.55);  // 10
    s.addNode( 3.95,	2.7,	0.46);  // 11
    s.addNode( 0.54,	0.35,	1.84);	// 12
    s.addNode( 4.18,	1.8,   -0.85);	// 13
    s.addNode(-1.22,	1.4,	0);	// 14
    s.addNode( 2.6,	3.1,   -2.43);	// 15
    s.addNode(-0.4,	2.7,	1.82);	// 16
    s.addNode( 3.65,	4.15,	0);	// 17
    s.addNode( 0.36,	4.15,	2.01);	// 18
    s.addNode( 0.68,	2.7,   -2.46);	// 19
    s.addNode( 2.95,	3.1,	2.13);	// 20
    s.addNode( 2.89,	1.4,   -2.25);	// 21
    s.addNode( 0.97,	1.8,    2.7);	// 22
    s.addNode( 0.54,	0.35,  -1.84);	// 23


}

void T12Model::addRods(tgStructure& s)
{
    s.addPair( 0,  1, "rod"); // 0
    s.addPair( 2,  3, "rod"); // 1
    s.addPair( 4,  5, "rod"); // 2
    s.addPair( 6,  7, "rod"); // 3
    s.addPair( 8,  9, "rod"); // 4
    s.addPair(10, 11, "rod"); // 5
    s.addPair(12, 13, "rod"); // 6
    s.addPair(14, 15, "rod"); // 7
    s.addPair(16, 17, "rod"); // 8
    s.addPair(18, 19, "rod"); // 9
    s.addPair(20, 21, "rod"); // 10
    s.addPair(22, 23, "rod"); // 11
}

void T12Model::addMuscles(tgStructure& s)
{
    // There are now two types of cables.
    // cableA corresponds to the unactuated cables on SUPERball, "passive."
    // cableB are the ones with motors attached, "active." (stiffer cables.)
    s.addPair(0, 12,  "cableB");
    s.addPair(0, 14,  "cableB");
    s.addPair(0, 23,  "cableB"); //muscle_active
   
    s.addPair(1, 8,   "cableB");
    s.addPair(1, 10,  "cableB");
    s.addPair(1, 18,  "cableB");

    s.addPair(2, 7,   "cableB");
    s.addPair(2, 9,   "cableB");
    s.addPair(2, 12,  "cableB");

    s.addPair(3, 17,  "cableB");
    s.addPair(3, 18,  "cableB");
    s.addPair(3, 20,  "cableB");

    s.addPair(4, 7,   "cableB");
    s.addPair(4, 21,  "cableB");
    s.addPair(4, 23,  "cableB");

    s.addPair(5, 10,  "cableB");
    s.addPair(5, 15,  "cableB");
    s.addPair(5, 17,  "cableB");

    s.addPair(6, 14,  "cableB");
    s.addPair(6, 19,  "cableB");
    s.addPair(6, 23,  "cableB");

    s.addPair(7, 13,  "cableB");

    s.addPair(8, 14,  "cableB");
    s.addPair(8, 16,  "cableB");

    s.addPair(9, 20,  "cableB");
    s.addPair(9, 22,  "cableB");

    s.addPair(10, 19,  "cableB");

    s.addPair(11, 13,  "cableB");
    s.addPair(11, 17,  "cableB");
    s.addPair(11, 20,  "cableB");

    s.addPair(12, 22,  "cableB");

    s.addPair(13, 21,  "cableB");

    s.addPair(15, 19,  "cableB");
    s.addPair(15, 21,  "cableB");

    s.addPair(16, 18,  "cableB");
    s.addPair(16, 22,  "cableB");

}

void T12Model::setup(tgWorld& world)
{
    // rod config: same for all rods on this model
    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);
    
    // cable configs for the two cables
    // muscleConfigA is for cableA, the "passive" one on SUPERball
    // @todo acceleration constraint deprecated, replace with tgKinematicActuator
    tgBasicActuator::Config muscleConfigA(c.stiffnessA, c.damping, c.pretensionA, 
					  c.hist, c.maxTens, c.targetVelocity);

    tgBasicActuator::Config muscleConfigB(c.stiffnessB, c.damping, c.pretensionB, 
					  c.hist, c.maxTens, c.targetVelocity);
            
    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addMuscles(s);
    s.move(btVector3(0, 10, 0));

    // Add a rotation. This is needed if the ground slopes too much,
    // otherwise  glitches put a rod below the ground.
    btVector3 rotationPoint = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis = btVector3(1, 1, 0.75);  // For landing on a square, use (1, 1, 0.5)
    double rotationAngle = M_PI/3;
    s.addRotation(rotationPoint, rotationAxis, rotationAngle);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("cableA", new tgBasicActuatorInfo(muscleConfigA));
    spec.addBuilder("cableB", new tgBasicActuatorInfo(muscleConfigB));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    // @todo check and confirm these calls actually work correctly.
    allMuscles = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());

    passiveMuscles = tgCast::find<tgModel, tgBasicActuator>(tgTagSearch("cableA"), 
							    getDescendants());
    activeMuscles = tgCast::find<tgModel, tgBasicActuator>(tgTagSearch("cableB"), 
							   getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void T12Model::step(double dt)
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

void T12Model::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

// Return the center of mass of this model
std::vector<double> T12Model::getBallCOM() {
    std::vector <tgRod*> rods = find<tgRod>("rod");
    assert(!rods.empty());
    btVector3 ballCenterOfMass(0, 0, 0);
    double ballMass = 0.0;
    for (std::size_t i = 0; i < 12; i++) {
        const tgRod* const rod = rods[i];
        assert(rod != NULL);
        const double rodMass = rod->mass();
        const btVector3 rodCenterOfMass = rod->centerOfMass();
        ballCenterOfMass += rodCenterOfMass * rodMass;
        ballMass += rodMass;
    }

    assert(ballMass > 0.0);
    ballCenterOfMass /= ballMass;

    // Copy to the result std::vector
    std::vector<double> result(3);
    for (size_t i = 0; i < 3; ++i) { result[i] = ballCenterOfMass[i]; }
    //std::cout<<"COM: (" << result[0] << ", " << result[1] << ", " << result[2] << ")\n";

    return result;
}

// Return the center of mass of rod
std::vector<double> T12Model::getRodCOM(int rodIndex) {
    std::vector <tgRod*> rods = find<tgRod>("rod");
    assert(!rods.empty());
    //cout << "Number of rods: " << rods.size() << endl;
    const tgRod* const rod = rods[rodIndex];
    assert(rod != NULL);
    btVector3 rodCenterOfMass = rod->centerOfMass();

    // Copy to the result std::vector
    std::vector<double> result(3);
    for (size_t i = 0; i < 3; ++i) { 
	result[i] = rodCenterOfMass[i]; 
    }
    return result;
}


// @TODO: Return the position of nodes connected to rod number rodIndex
std::vector<double> T12Model::getNodePosition(int rodIndex) {
    std::vector <tgRod*> rods = find<tgRod>("rod");
    const tgRod* const rod = rods[rodIndex];
    btVector3 rodCOM = rod->centerOfMass();
    btScalar x = rodCOM[0];
    btScalar y = rodCOM[1];
    btScalar z = rodCOM[2];
    
    btMatrix3x3 rotation = btMatrix3x3(((tgBaseRigid*) rod)->getPRigidBody()->getOrientation());
    btVector3 orig1 = btVector3(0, (rod->length())/2, 0); // Distance between rod COM and node1
    btVector3 orig2 = btVector3(0, -(rod->length())/2, 0); // Distance between rod COM and node2

    btVector3 pos1 = (rotation * orig1) + rodCOM; // position of node1 
    btVector3 pos2 = (rotation * orig2) + rodCOM; // position of node2

    // Copy to the result std::vector
    std::vector<double> result(6);
    for (size_t i = 0; i < 3; ++i) { 
	result[i] = pos1[i];
	result[i+3] = pos2[i]; 
    }	
//    cout << "Node position 1: (" << result[0] << ", " << result[1] << ", " << result[2] << ")\n";
//    cout << "Node position 2: (" << result[3] << ", " << result[4] << ", " << result[5] << ")\n";

    return result;
}


const std::vector<tgBasicActuator*>& T12Model::getAllMuscles() const
{
    return allMuscles;
}

const std::vector<tgBasicActuator*>& T12Model::getPassiveMuscles() const
{
    return passiveMuscles;
}

const std::vector<tgBasicActuator*>& T12Model::getActiveMuscles() const
{
    return activeMuscles;
}

const double T12Model::muscleRatio()
{
	return (c.stiffnessA/c.stiffnessB);
	//return 2.5;
}
    
void T12Model::teardown()
{
    cout << "Tearing down" << endl;
    tgModel::teardown();
}

