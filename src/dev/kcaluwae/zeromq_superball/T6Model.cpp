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
 * @file T6Model.cpp
 * @brief Contains the implementation of class T6Model.
 * $Id$
 */

// This module
#include "T6Model.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgKinematicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgKinematicActuatorInfo.h"
#include "tgcreator/tgBasicContactCableInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
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
    // "mp" stands for middle part. The specs without this extension are for end caps.
    const struct Config
    {
        double density;
        double radius;
        double density_mp;
        double radius_mp;
        double stiffnessPassive;
	double stiffnessActive;
        double damping;
        double rod_length;
        double rod_space;
        double rod_length_mp;
        double friction;
        double rollFriction;
        double restitution;
        double pretensionPassive;
	double pretensionActive;
        bool   hist;
        double maxTens;
        double targetVelocity;
        double motor_radius;
        double motor_friction;
        double motor_inertia;
        bool   backDrivable;
    } c =
   {
     0.38618,    // density (kg / length^3) weight of both endcaps: 3.3kg (1.65 kg each)
     0.35,     // radius (length) radius of an endcap
     0.208,      // density_mp (kg / length^3) weight of connecting rod: 200g 
     0.175,      //radius_mp (length) radius of the connecting rod
     998.25,   // stiffnessPassive (kg / sec^2)
     3152.36,  // stiffnessActive (kg / sec^2)
     200.0,    // damping (kg / sec)
     17.4,     // rod_length (length)
     17.4/4,//4.5,      // rod_space (length)
     17.4/2,        // rod_length_mp (length)
     0.99,      // friction (unitless)
     0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)
     100.0,    // pretension -> set to 
     100.0,   // pretension -> set to 
     0,			// History logging (boolean)
     4000,   // maxTens
     2,    // targetVelocity
     0.09, // motor_radius // Spindle radius (length)
     2*4.24e-5, // motor_friction (kg*(length)^2/sec)
     4*2.749e-4, // motor_inertia (kg*(length)^2) // Inertia of motor, gearbox, and spindle all together
     0, // Not backDrivable

     // Use the below values for earlier versions of simulation.
     // 1.006,
     // 0.31,
     // 300000.0,
     // 3000.0,
     // 15.0,
     // 7.5,
  };
} // namespace

T6Model::T6Model() : tgModel()
{
}

T6Model::~T6Model()
{
}

void T6Model::addNodes(tgStructure& s)
{
    const double half_length = c.rod_length / 2;
    const double half_length_mp = c.rod_length_mp / 2;
/* OLD CODE!
    s.addNode(-c.rod_space,  -half_length, 0);            // 1
    s.addNode(-c.rod_space,   half_length, 0);            // 2
    s.addNode( c.rod_space,  -half_length, 0);            // 3
    s.addNode( c.rod_space,   half_length, 0);            // 4
    s.addNode(0,           -c.rod_space,   -half_length); // 5
    s.addNode(0,           -c.rod_space,    half_length); // 6
    s.addNode(0,            c.rod_space,   -half_length); // 7
    s.addNode(0,            c.rod_space,    half_length); // 8
    s.addNode(-half_length, 0,            c.rod_space);   // 9
    s.addNode( half_length, 0,            c.rod_space);   // 10
    s.addNode(-half_length, 0,           -c.rod_space);   // 11
    s.addNode( half_length, 0,           -c.rod_space);   // 12
*/

/* MATLAB Code from UKF by Jeff F. for reference
 * This is correct
 * nodes = [
    barSpacing      0               -barLength*0.5;
    barSpacing      0                barLength*0.5;
    0               barLength*0.5    barSpacing;   %5
    0              -barLength*0.5    barSpacing;
    barLength*0.5   barSpacing       0;
   -barLength*0.5   barSpacing       0;
   -barSpacing      0                barLength*0.5;
   -barSpacing      0               -barLength*0.5;             %8      
    0               barLength*0.5   -barSpacing;
    0              -barLength*0.5   -barSpacing;
    barLength*0.5  -barSpacing       0;
   -barLength*0.5  -barSpacing       0;
    ];

*/   

// **** This matches the robot. "Z" is negated from what is in the UKF solver **** 
    s.addNode( c.rod_space,    0,              half_length);	// 1
    s.addNode( c.rod_space,    0,             -half_length);	// 2
    s.addNode( 0,              half_length,   -c.rod_space);	// 3
    s.addNode( 0,             -half_length,   -c.rod_space);	// 4
    s.addNode( half_length,    c.rod_space,    0);		// 5
    s.addNode(-half_length,    c.rod_space,    0);		// 6
    s.addNode(-c.rod_space,    0,             -half_length);	// 7
    s.addNode(-c.rod_space,    0,              half_length);	// 8
    s.addNode( 0,              half_length,    c.rod_space); 	// 9
    s.addNode( 0,             -half_length,    c.rod_space); 	// 10
    s.addNode( half_length,   -c.rod_space,    0);		// 11
    s.addNode(-half_length,   -c.rod_space,    0);   		// 12
/*    
    s.addNode( c.rod_space,   -half_length,   0);	// 1
    s.addNode( c.rod_space,    half_length,    0);	// 2
    s.addNode( 0,              c.rod_space,    half_length);	// 3
    s.addNode( 0,              c.rod_space,   -half_length);	// 4
    s.addNode( half_length,    0,              c.rod_space);		// 5
    s.addNode(-half_length,    0,              c.rod_space);		// 6
    s.addNode(-c.rod_space,    half_length,    0);	// 7
    s.addNode(-c.rod_space,   -half_length,    0);	// 8
    s.addNode( 0,             -c.rod_space,    half_length); 	// 9
    s.addNode( 0,             -c.rod_space,   -half_length); 	// 10
    s.addNode( half_length,    0,             -c.rod_space);		// 11
    s.addNode(-half_length,    0,             -c.rod_space);   		// 12
*/
/*    
    s.addNode( c.rod_space,    0,             -half_length_mp);	// 13(1-2)
    s.addNode( c.rod_space,    0,              half_length_mp);	// 14(2-1)
    s.addNode( 0,              half_length_mp, c.rod_space);	// 15(3-4)
    s.addNode( 0,             -half_length_mp, c.rod_space);	// 16(4-3)
    s.addNode( half_length_mp, c.rod_space,    0);		// 17(5-6)
    s.addNode(-half_length_mp, c.rod_space,    0);		// 18(6-5)
    s.addNode(-c.rod_space,    0,              half_length_mp);	// 19(7-8)
    s.addNode(-c.rod_space,    0,             -half_length_mp);	// 20(8-7)
    s.addNode( 0,              half_length_mp,-c.rod_space); 	// 21(9-10)
    s.addNode( 0,             -half_length_mp,-c.rod_space); 	// 22(10-9)
    s.addNode( half_length_mp,-c.rod_space,    0);   		// 23(11-12)
    s.addNode(-half_length_mp,-c.rod_space,    0);   		// 24(12-11)
*/
}

void T6Model::addRods(tgStructure& s)
{
    s.addPair( 0, 1, "rod");
    s.addPair( 2, 3, "rod");
    s.addPair( 4, 5, "rod");
    s.addPair( 6, 7, "rod");
    s.addPair( 8, 9, "rod");
    s.addPair( 10, 11, "rod");
/*
    s.addPair( 0,  12, "rod");
    s.addPair( 1,  13, "rod");
    s.addPair( 12,  13, "rodmp");   //middle part
    s.addPair( 2,  14, "rod");
    s.addPair( 3,  15, "rod");
    s.addPair( 14,  15, "rodmp");   //middle part
    s.addPair( 4,  16, "rod");
    s.addPair( 5,  17, "rod");
    s.addPair( 16,  17, "rodmp");   //middle part
    s.addPair( 6,  18, "rod");
    s.addPair( 7,  19, "rod");
    s.addPair( 18,  19, "rodmp");   //middle part
    s.addPair( 8,  20, "rod");
    s.addPair( 9,  21, "rod");
    s.addPair( 20,  21, "rodmp");   //middle part
    s.addPair( 10, 22, "rod");
    s.addPair( 11, 23, "rod");
    s.addPair( 22,  23, "rodmp");   //middle part
*/
}

void T6Model::addActuators(tgStructure& s)
{
	/* MATLAB Code from UKF by Jeff F. for reference
	 * This is correct - Indices start at 1 for MATLAB, 0 for NTRT code
	 strings = [1  2 3 4 5 6 7 8  9 11 12 10   1 1 11 11 10 10 3 3 7  7 6 6;
	            11 5 7 2 9 3 6 12 8 10 4  1    9 5  2  4  12  8 5 2 4 12 8 9];
				* NTRT Conversion below *
			   [0  1 2 3 4 5 6 7  8 10 11 9   0 0 10 10  9   9 2 2 6  6  5 5;
                            10 4 6 1 8 2 5 11 7 9  3  0   8 4  1  3  11  7 4 1 3  11 7 8];

	*/
        s.addPair(0,    10,     "actuated");
        s.addPair(1,    4,      "actuated");
        s.addPair(2,    6,      "actuated");
        s.addPair(3,    1,      "actuated");
        s.addPair(4,    8,      "actuated");
        s.addPair(5,    2,      "actuated");
        s.addPair(6,    5,      "actuated");
        s.addPair(7,    11,     "actuated");
        s.addPair(8,    7,      "actuated");
        s.addPair(9,    0,      "actuated");
        s.addPair(10,   9,      "actuated");
        s.addPair(11,   3,      "actuated");

        //Passive cables (triangle 1 - large circle)
        s.addPair(0,    8,      "passive");
        s.addPair(0,    4,      "passive");
        s.addPair(10,   1,      "passive");
        s.addPair(10,   3,      "passive");
        s.addPair(9,    11,     "passive");
        s.addPair(9,    7,      "passive");

        //Passive cables (triangle 2 - large circle)
        s.addPair(2,    4,      "passive");
        s.addPair(2,    1,      "passive");
        s.addPair(6,    3,      "passive");
        s.addPair(6,    11,     "passive");
        s.addPair(5,    7,      "passive");
        s.addPair(5,    8,      "passive");

	/*
        THIS IS THE CORRECT PATTERN, ABOVE IS PUTTING THE ACTUATORS IT IN ORDER 
        SO THAT WE CAN ITTERATE THROUGH THE ARRAY CORRECTLY
	//Large circle
	s.addPair(1,	4,	"actuated");
	s.addPair(4,	8,	"actuated");
	s.addPair(8,	7,	"actuated");
	s.addPair(7,	11,	"actuated");
	s.addPair(11,	3,	"actuated");
	s.addPair(3,	1,	"actuated");

	//Triangle 1
	s.addPair(0,	10,	"actuated");
	s.addPair(10,	9,	"actuated");
	s.addPair(9,	0, 	"actuated");

	//Triangle 2
	s.addPair(2,	6,	"actuated");
	s.addPair(6,	5,	"actuated");
	s.addPair(5,	2,	"actuated");

	//Passive cables (triangle 1 - large circle)
	s.addPair(0,	8,	"passive");
	s.addPair(0,	4,	"passive");
	s.addPair(10,	1,	"passive");
	s.addPair(10,	3,	"passive");
	s.addPair(9,	11,	"passive");
	s.addPair(9,	7,	"passive");

	//Passive cables (triangle 2 - large circle)
	s.addPair(2,	4,	"passive");
	s.addPair(2,	1,	"passive");
	s.addPair(6,	3,	"passive");
	s.addPair(6,	11,	"passive");
	s.addPair(5,	7,	"passive");
	s.addPair(5,	8,	"passive");

    s.addPair(0, 4,  "passive");//OK
    s.addPair(0, 5,  "passive");//
    s.addPair(0, 8,  "passive");
    s.addPair(0, 10, "passive");

    s.addPair(1, 6,  "passive");
    s.addPair(1, 7,  "passive");
    s.addPair(1, 8,  "passive");
    s.addPair(1, 10, "passive");

    s.addPair(2, 4,  "passive");
    s.addPair(2, 5,  "passive");
    s.addPair(2, 9,  "passive");
    s.addPair(2, 11, "passive");

    s.addPair(3, 7,  "passive");
    s.addPair(3, 6,  "passive");
    s.addPair(3, 9,  "passive");
    s.addPair(3, 11, "passive");

    s.addPair(4, 2,  "passive");
    s.addPair(4, 10, "passive");
    s.addPair(4, 11, "passive");

    s.addPair(5, 8,  "passive");
    s.addPair(5, 9,  "passive");

    s.addPair(6, 10, "passive");
    s.addPair(6, 11, "passive");

    s.addPair(7, 8,  "passive");
    s.addPair(7, 9,  "passive");
*/
}

void T6Model::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction,
				c.rollFriction, c.restitution);

    // This part is added by Ali to make a more accurate model of SuperBall's Rods
    //const tgRod::Config rodConfigmp(c.radius_mp, c.density_mp, c.friction,
      //          c.rollFriction, c.restitution);

    /// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate
    //tgBasicActuator::Config actuatedCableConfig(3100., c.damping, c.pretension, c.hist, c.maxTens, c.targetVelocity);
    tgBasicActuator::Config passiveCableConfig(c.stiffnessPassive, c.damping, c.pretensionPassive, c.hist,
					    c.maxTens, c.targetVelocity);
    // This part is added by Ali to make a more accurate model of SuperBall's Rods
    tgBasicActuator::Config motorConfig(c.stiffnessActive, c.damping, c.pretensionActive, c.hist,
    					    c.maxTens, c.targetVelocity);
    
    //tgKinematicActuator::Config motorConfig(c.stiffnessActive, c.damping, c.pretensionActive, c.motor_radius, c.motor_friction,
    //                    c.motor_inertia, c.backDrivable, c.hist, c.maxTens, c.targetVelocity);
    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addActuators(s);
    s.move(btVector3(0, (c.rod_length/2)-1, 0));

    // Add a rotation. This is needed if the ground slopes too much,
    // otherwise  glitches put a rod below the ground.
    btVector3 rotationPoint = btVector3(0, (c.rod_length/2), 0); // origin
    btVector3 rotationAxis = btVector3(0, 0, 1);  // y-axis
    //double rotationAngle = M_PI/2;
    double rotationAngle = -0.8;
    s.addRotation(rotationPoint, rotationAxis, rotationAngle);

    rotationAxis = btVector3(1, 0, 0);  // y-axis
    rotationAngle = -0.6;
    s.addRotation(rotationPoint, rotationAxis, rotationAngle);

    //rotationAxis = btVector3(0, 0, 1);  // y-axis
    //rotationAngle = 1.7;
    //s.addRotation(rotationPoint, rotationAxis, rotationAngle);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    //spec.addBuilder("rodmp", new tgRodInfo(rodConfigmp));
    //spec.addBuilder("actuated", new tgKinematicActuatorInfo(motorConfig));
    spec.addBuilder("actuated", new tgBasicContactCableInfo(motorConfig));
    spec.addBuilder("passive", new tgBasicContactCableInfo(passiveCableConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control.
    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
    //allActuators = this->find<tgKinematicActuator> ("actuated");

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void T6Model::step(double dt)
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

void T6Model::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& T6Model::getAllActuators() const
{
    return allActuators;
}

void T6Model::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
