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
 * @file v3Model.cpp
 * @brief Contains the implementation of class v3Model.
 * $Id$
 */

// This module
#include "v3Model.h"
#include "v3TensionController.h"
// This library
#include "core/tgBasicActuator.h"
#include "controllers/tgTensionController.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgKinematicContactCableInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
//#include "core/abstractMarker.h"
// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>

namespace
{
    // see tgBasicActuator and tgRod for a descripton of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    // Note: This current model of the SUPERball rod is 1.5m long by 3 cm radius,
    // which is 0.00424 m^3.
    // For SUPERball v1.5, mass = 3.5kg per strut, which comes out to 
    // 0.825 kg / (decimeter^3).

    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        double density;
        double density_motor;
        double radius;
	double radius_motor;
        double stiffness;
        double damping;
        double rod_length;
        double rod_space;    
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        bool   hist;
        double maxTens;
        double targetVelocity;
    } c =
   {
     0.688,    // density of rod (kg / length^3)
     0.751,    // density of motor (kg/ length^3)
     0.127/2,     // radius (length) ** rod diameter / 2 **
     0.56/2,     // radius (length) ** motor diameter / 2 **
     200.0,   // stiffness (kg / sec^2) was 1500
     20.0,    // damping (kg / sec)
     6.5,     // rod_length (length)
     3.25,      // rod_space (length)
     0.99,      // friction (unitless)
     0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)
     175.0,        // pretension -> scaled to 85.0 which is 10 times the actual scaling because gravity is 10 times higher
     0,			// History logging (boolean)
     100000,   // maxTens
     10000,    // targetVelocity

     // Use the below values for earlier versions of simulation.
     // 1.006,    
     // 0.31,     
     // 300000.0, 
     // 3000.0,   
     // 15.0,     
     // 7.5,      
  };
} // namespace

v3Model::v3Model() : tgModel() 
{
}

v3Model::~v3Model()
{
}

void v3Model::addNodes(tgStructure& s)
{
    const double third_length = 2.26155; //226.155 mm
    const double motor_length = 1.9769; //197.69 mm
    const size_t nNodes = 26;


    // lower rod: ROD 1
    nodePositions.push_back(btVector3(0, -c.rod_space, -third_length - motor_length/2)); //0
    nodePositions.push_back(btVector3(0, -c.rod_space, -motor_length/2)); //1
    nodePositions.push_back(btVector3(0, -c.rod_space, motor_length/2)); //2
    nodePositions.push_back(btVector3(0, -c.rod_space, third_length + motor_length/2)); //3

    // upper rod: ROD 2
    nodePositions.push_back(btVector3(0, c.rod_space, -third_length - motor_length/2)); //4
    nodePositions.push_back(btVector3(0, c.rod_space, -motor_length/2)); //5
    nodePositions.push_back(btVector3(0, c.rod_space, motor_length/2)); //6
    nodePositions.push_back(btVector3(0, c.rod_space, third_length + motor_length/2)); //7

    //right rod: ROD 4
    nodePositions.push_back(btVector3(-third_length- motor_length/2, 0, c.rod_space)); //8
    nodePositions.push_back(btVector3(-motor_length/2, 0, c.rod_space)); //9
    nodePositions.push_back(btVector3(motor_length/2, 0, c.rod_space)); //10
    nodePositions.push_back(btVector3(third_length + motor_length/2, 0, c.rod_space)); //11

    //left rod: ROD 3
    nodePositions.push_back(btVector3(-third_length- motor_length/2, 0, -c.rod_space)); //12
    nodePositions.push_back(btVector3(-motor_length/2, 0, -c.rod_space)); //13
    nodePositions.push_back(btVector3(motor_length/2, 0, -c.rod_space)); //14
    nodePositions.push_back(btVector3(third_length + motor_length/2, 0, -c.rod_space)); //15

    //center rod close: ROD 6
    nodePositions.push_back(btVector3(-c.rod_space, -third_length - motor_length/2, 0)); // 16
    nodePositions.push_back(btVector3(-c.rod_space, -motor_length/2, 0)); // 17
    nodePositions.push_back(btVector3(-c.rod_space, motor_length/2, 0)); // 18
    nodePositions.push_back(btVector3(-c.rod_space, third_length + motor_length/2, 0)); // 19

    //center rod far: ROD 5 
    nodePositions.push_back(btVector3(c.rod_space, -third_length - motor_length/2, 0)); // 20
    nodePositions.push_back(btVector3(c.rod_space, -motor_length/2, 0)); // 21
    nodePositions.push_back(btVector3(c.rod_space, motor_length/2, 0)); // 22
    nodePositions.push_back(btVector3(c.rod_space, third_length + motor_length/2, 0)); // 23
	
    //Nodes of TRACKER in center of ROD 2. (If using other tracker comment the follow two lines)
    //nodePositions.push_back(btVector3(0, c.rod_space-0.5, (-third_length - motor_length/2)/20));//24
    //nodePositions.push_back(btVector3(0, c.rod_space-0.5, (third_length + motor_length/2)/20)); //25

    //Nodes of TRACKER in center of ROD 2. (If using other tracker comment the follow two lines)
    nodePositions.push_back(btVector3(-c.rod_space, third_length + motor_length/2, 0)); // 24
    nodePositions.push_back(btVector3(-c.rod_space+ .01 , third_length + motor_length/2 + .01, 0)); // 25

	for(size_t i=0;i<nNodes;i++) {
		s.addNode(nodePositions[i][0],nodePositions[i][1],nodePositions[i][2]);
	}

}

void v3Model::addRods(tgStructure& s)
{
    //ROD 1 
    s.addPair( 0,  1, "rod");
    s.addPair( 1,  2, "motor");
    s.addPair( 2,  3, "rod");	
 
    //ROD 2
    s.addPair( 4,  5, "rod");
    s.addPair( 5,  6, "motor");
    s.addPair( 6,  7, "rod");

    s.addPair( 8,  9, "rod");
    s.addPair( 9,  10, "motor");
    s.addPair( 10,  11, "rod");

    s.addPair( 12,  13, "rod");
    s.addPair( 13,  14, "motor");
    s.addPair( 14,  15, "rod");

    //ROD 5
    s.addPair( 20,  21, "rod");
    s.addPair( 21,  22, "motor");
    s.addPair( 22,  23, "rod");

    //ROD 6
    s.addPair( 16,  17, "rod");
    s.addPair( 17,  18, "motor");
    s.addPair( 18,  19, "rod");

    //TRACKER UPPER BAR ROD 2 (comment out if using other tracker)
    //s.addPair( 24,  25, "rod endeffector");

    //TRACKER TOP NODES (Node 19 in NTRT, Top of Rod 6 that has cables 6.3, 6.4, 4.3, and 2.4)
    s.addPair( 24, 25, "eeRod endeffector");
}

void v3Model::addMuscles(tgStructure& s)
{

    /*s.addPair(16, 0,  "muscle"); //cable: 6.2
    s.addPair(16, 3,  "muscle"); //cable: 1.2
    s.addPair(16, 8,  "muscle"); 
    s.addPair(16, 12, "muscle");

    s.addPair(19, 4,  "muscle");
    s.addPair(19, 7,  "muscle");
    s.addPair(19, 8,  "muscle");
    s.addPair(19, 12, "muscle");

    s.addPair(20, 0,  "muscle");
    s.addPair(20, 3,  "muscle");
    s.addPair(20, 11,  "muscle");
    s.addPair(20, 15, "muscle");

    s.addPair(23, 7,  "muscle");
    s.addPair(23, 4,  "muscle");
    s.addPair(23, 11,  "muscle");
    s.addPair(23, 15, "muscle");

    s.addPair(0, 20,  "muscle");
    s.addPair(0, 12, "muscle");
    s.addPair(0, 15, "muscle");

    s.addPair(3, 8,  "muscle");
    s.addPair(3, 11,  "muscle");

    s.addPair(4, 12, "muscle");
    s.addPair(4, 15, "muscle");

    s.addPair(7, 8,  "muscle");
    s.addPair(7, 11, "muscle");*/

    s.addPair(11, 3,  "muscle"); //cable: 1.1
    s.addPair(3, 16,  "muscle"); //cable: 1.2
    s.addPair(20, 0,  "muscle"); //cable: 1.3
    s.addPair(12, 0,  "muscle"); //cable: 1.4

    s.addPair(7, 23,  "muscle"); //cable: 2.1
    s.addPair(7, 8,  "muscle"); //cable: 2.2
    s.addPair(4, 15,  "muscle"); //cable: 2.3
    s.addPair(4, 19,  "muscle"); //cable: 2.4

    s.addPair(15, 23,  "muscle"); //cable: 3.1
    s.addPair(15, 0,  "muscle"); //cable: 3.2
    s.addPair(4, 12,  "muscle"); //cable: 3.3
    s.addPair(12, 16,  "muscle"); //cable: 3.4

    s.addPair(7, 11,  "muscle"); //cable: 4.1
    s.addPair(20, 11,  "muscle"); //cable: 4.2
    s.addPair(19, 8,  "muscle"); //cable: 4.3
    s.addPair(3, 8,  "muscle"); //cable: 4.4

    s.addPair(3, 20,  "muscle"); //cable: 5.1
    s.addPair(15, 20,  "muscle"); //cable: 5.2
    s.addPair(11, 23,  "muscle"); //cable: 5.3
    s.addPair(4, 23,  "muscle"); //cable: 5.4

    s.addPair(16, 8,  "muscle"); //cable: 6.1
    s.addPair(16, 0,  "muscle"); //cable: 6.2
    s.addPair(19, 7,  "muscle"); //cable: 6.3
    s.addPair(19, 12,  "muscle"); //cable: 6.4

    //use if tracker is set to center of Rod 2
   //s.addPair(24, 5, "ee");  
   // s.addPair(25, 6, "ee");


}

void v3Model::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);
    const tgRod::Config motorConfig(c.radius_motor, c.density_motor, c.friction, 
				c.rollFriction, c.restitution);
    const tgRod::Config eeRodConfig(c.radius, 0.396, c.friction, 
				c.rollFriction, c.restitution);
    //const tgRod::Config EEConfig(0.001, 0.01, c.friction, 
				//c.rollFriction, c.restitution);
    /// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate
    tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension, c.hist, 
					    c.maxTens, c.targetVelocity);
    tgBasicActuator::Config eeConfig(250.0, 10.0, 5.0, c.hist, 
					    c.maxTens, c.targetVelocity);        
    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addMuscles(s);

    btVector3 offset (-10.1/2, 50.0, -12.1/2);
    s.move(offset);

    // Add a rotation. This is needed if the ground slopes too much,
    // otherwise  glitches put a rod below the ground.
    btVector3 rotationPoint = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis = btVector3(0.5, 1, 0);  // y-axis
    double rotationAngle = M_PI/2;
    s.addRotation(rotationPoint, rotationAxis, rotationAngle);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
    // added 8/10/15 
    spec.addBuilder("motor", new tgRodInfo(motorConfig));

    // added  10/21/15
    //spec.addBuilder("EE", new tgRodInfo(EEConfig));
    spec.addBuilder("ee", new tgBasicActuatorInfo(eeConfig));
    spec.addBuilder("eeRod", new tgRodInfo(eeRodConfig));
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void v3Model::step(double dt)
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

void v3Model::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& v3Model::getAllMuscles() const
{
    return allMuscles;
}
    
void v3Model::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
