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
 * @file sixBarModel.cpp
 * @brief Contains the definition of the members of the class sixBarModel.
 * $Id$
 */

// This Module
#include "sixBarModel.h"
// C++ Standard Libraries
#include <math.h>
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// Utility Library
#include "../utility.hpp"

namespace 
{
	/**
   * Configuration parameters so they're easily accessable.
   * All parameters must be positive.
   */

   double sf = 10;

   const struct Config 
   {
   		double density;
   		double radius;
   		double stiffness;
   		double damping;
   		double rodLength;
   		double friction;
   		double rollFriction;
   		double restitution;
   		double pretension;
   		bool hist;
   		double maxTension;
   		double targetVelocity;
   		double motorLength;
   	} config =
   		{
   			/*
   			// SuperBall parameters
   			688/pow(sf,3),    // density (kg / length^3)
	        0.031*sf,     // radius (length)
	        1615.0,   // stiffness (kg / sec^2) was 1500
	        200.0,    // damping (kg / sec)
	        1.684*sf,     // rodLength (length)
	        0.99,      // friction (unitless)
	        0.01,     // rollFriction (unitless)
	        0.0,      // restitution (?)
	        100.0*sf,        // pretension -> set to 4 * 613, the previous value of the rest length controller
	        0,         // History logging (boolean)
	        10000*sf,   // maxTension
	        0.5*sf,    // targetVelocity
	        0.3175*sf, // Motor length
	        */
	        
	        688/pow(sf,3),    // density (kg / length^3)
	        0.019*sf,     // radius (length)
	        400.0,   // stiffness (kg / sec^2) was 1500
	        20.0,    // damping (kg / sec)
	        1.684*sf,     // rodLength (length)
	        0.99,      // friction (unitless)
	        0.01,     // rollFriction (unitless)
	        0.0,      // restitution (?)
	        100.0*sf,        // pretension -> set to 4 * 613, the previous value of the rest length controller
	        0,         // History logging (boolean)
	        10000*sf,   // maxTension
	        0.5*sf,    // targetVelocity
	        0.3175*sf, // Motor length

	        /*
	        //TT4-Mini parameters
	        2485/pow(sf,3),
	        0.0045*sf,
	        46.173,
	        200.0,
	        0.245*sf,
	        0.99,
	        0.01,
	        0.0,
	        0.7*sf,
	        0,
	        10000*sf,
	        0.3*sf,
	        */
   		};
}

sixBarModel::sixBarModel() : tgModel() 
{

}

sixBarModel::~sixBarModel()
{
}

void sixBarModel::addSixBar(tgStructure& s)
{
	addSixBarNodes(s);
	addSixBarRods(s);
	addSixBarActuators(s);
}

void sixBarModel::addPayload(tgStructure& s)
{
	addPayloadNodes(s);
	addPayloadRods(s);
	addPayloadStrings(s);
}

void sixBarModel::setup(tgWorld& world)
{
	// Calculate the space between two parallel rods based on the rod length from Config
	rodDist = (-config.rodLength + sqrt(pow(config.rodLength,2)+4*pow(config.rodLength,2)))/2;

	// Nodes in the x-z plane
	node0 = btVector3(-rodDist/2, 0, config.rodLength/2); // 0
	node1 = btVector3(-rodDist/2, 0, -config.rodLength/2); // 1
	node2 = btVector3(rodDist/2, 0, -config.rodLength/2); // 2
	node3 = btVector3(rodDist/2, 0, config.rodLength/2); // 3

	// Nodes in the y-z plane
	node4 = btVector3(0, -config.rodLength/2, rodDist/2); // 4
	node5 = btVector3(0, config.rodLength/2, rodDist/2); // 5
	node6 = btVector3(0, config.rodLength/2, -rodDist/2); // 6
	node7 = btVector3(0, -config.rodLength/2, -rodDist/2); // 7

	// Nodes in the x-y plane
	node8 = btVector3(-config.rodLength/2, -rodDist/2, 0); // 8
	node9 = btVector3(-config.rodLength/2, rodDist/2, 0); // 9
	node10 = btVector3(config.rodLength/2, rodDist/2, 0); // 10
	node11 = btVector3(config.rodLength/2, -rodDist/2, 0); // 11

	// Find all edge vectors of closed triangles	// Actuator #
	face0Edge0 = node8 - node4;		// 16
	face0Edge1 = node0 - node8;		// 2
	face0Edge2 = node4 - node0;		// 0

	face2Edge0 = node9 - node0;		// 3
	face2Edge1 = node5 - node9;		// 18
	face2Edge2 = node0 - node5;		// 1

	face5Edge0 = node3 - node4;		// 12
	face5Edge1 = node11 - node3;	// 15
	face5Edge2 = node4 - node11;	// 17

	face7Edge0 = node5 - node3;		// 13
	face7Edge1 = node10 - node5;	// 19
	face7Edge2 = node3 - node10;	// 14

	face8Edge0 = node11 - node7;	// 23
	face8Edge1 = node2 - node11;	// 11
	face8Edge2 = node7 - node2;		// 9

	face10Edge0 = node10 - node2;	// 10
	face10Edge1 = node6 - node10;	// 21
	face10Edge2 = node2 - node6;	// 8

	face13Edge0 = node1 - node7;	// 5
	face13Edge1 = node8 - node1;	// 6
	face13Edge2 = node7 - node8;	// 22

	face15Edge0 = node6 - node1;	// 4
	face15Edge1 = node9 - node6;	// 20
	face15Edge2 = node1 - node9;	// 7

	// Find normal vectors to all faces
	face0Norm = (face0Edge0.cross(face0Edge2)).normalize();
	face1Norm = (face0Edge1.cross(face2Edge0)).normalize();
	face2Norm = (face2Edge0.cross(face2Edge2)).normalize();
	face3Norm = (face7Edge0.cross(face2Edge2)).normalize();
	face4Norm = (face0Edge2.cross(face5Edge0)).normalize();
	face5Norm = (face5Edge0.cross(face5Edge2)).normalize();
	face6Norm = (face7Edge2.cross(face5Edge1)).normalize();
	face7Norm = (face7Edge0.cross(face7Edge2)).normalize();

	face8Norm = (face8Edge0.cross(face8Edge2)).normalize();
	face9Norm = (face8Edge1.cross(face10Edge0)).normalize();
	face10Norm = (face10Edge0.cross(face10Edge2)).normalize();
	face11Norm = (face15Edge0.cross(face10Edge2)).normalize();
	face12Norm = (face8Edge2.cross(face13Edge0)).normalize();
	face13Norm = (face13Edge0.cross(face13Edge2)).normalize();
	face14Norm = (face15Edge2.cross(face13Edge1)).normalize();
	face15Norm = (face15Edge0.cross(face15Edge2)).normalize();

	face16Norm = (face0Edge0.cross(face13Edge2)).normalize();
	face17Norm = (face15Edge1.cross(face2Edge1)).normalize();
	face18Norm = (face7Edge1.cross(face10Edge1)).normalize();
	face19Norm = (face8Edge0.cross(face5Edge2)).normalize();

	// Place all normal vectors into vector
	normalVectors.push_back(face0Norm);
	normalVectors.push_back(face1Norm);
	normalVectors.push_back(face2Norm);
	normalVectors.push_back(face3Norm);
	normalVectors.push_back(face4Norm);
	normalVectors.push_back(face5Norm);
	normalVectors.push_back(face6Norm);
	normalVectors.push_back(face7Norm);
	normalVectors.push_back(face8Norm);
	normalVectors.push_back(face9Norm);
	normalVectors.push_back(face10Norm);
	normalVectors.push_back(face11Norm);
	normalVectors.push_back(face12Norm);
	normalVectors.push_back(face13Norm);
	normalVectors.push_back(face14Norm);
	normalVectors.push_back(face15Norm);
	normalVectors.push_back(face16Norm);
	normalVectors.push_back(face17Norm);
	normalVectors.push_back(face18Norm);
	normalVectors.push_back(face19Norm);

	// Define the configurations of the rods and strings
  	// Note that pretension is defined for this string
	const tgRod::Config sixBarRodConfig(config.radius, config.density, config.friction,
		config.rollFriction, config.restitution);
	const tgRod::Config PayloadRodConfig(config.radius*4, config.density/16, config.friction,
		config.rollFriction, config.restitution);
	const tgRod::Config motorConfig(config.radius*2, config.density, config.friction,
		config.rollFriction, config.restitution);

	tgBasicActuator::Config actuatorConfig(config.stiffness, config.damping, config.pretension,
		config.hist, config.maxTension, config.targetVelocity);
	tgBasicActuator::Config staticCableConfig(config.stiffness, config.damping, config.pretension*2,
		config.hist, config.maxTension, config.targetVelocity);

	// Create a structure that will hold the details of this model
	tgStructure s;

	// Add in the tensegrity structure
	addSixBar(s);

	// Add in the payload
	// addPayload(s);

	// Move the structure
	rotateToFace(s, 2);
	s.move(btVector3(0, 50, -0)); 
	//s.move(btVector3(100, 3420,-100));
	// -8 for 0.26, -9 for 0.25, 
	// s.move(btVector3(0, config.rodLength-9, 0));
	//s.move(btVector3(0, config.rodLength, 0));

	// Create the build spec that uses tags to turn the structure into a real model
	tgBuildSpec spec;
	spec.addBuilder("rod", new tgRodInfo(sixBarRodConfig));
	spec.addBuilder("payload", new tgRodInfo(PayloadRodConfig));
	spec.addBuilder("actuator", new tgBasicActuatorInfo(actuatorConfig));
	spec.addBuilder("cable", new tgBasicActuatorInfo(staticCableConfig));
	spec.addBuilder("motor", new tgRodInfo(motorConfig));

	// Create the structureInfo
	tgStructureInfo structureInfo(s, spec);
	structureInfo.buildInto(*this, world);

	// Get the rod rigid bodies for controller
	std::vector<tgRod*> rods = sixBarModel::find<tgRod>("rod");
	for (int i = 0; i < rods.size(); i++) {
		allRods.push_back(sixBarModel::find<tgRod>(tgString("rod num", i))[0]);
	}

	// Get the actuators for controller
	std::vector<tgBasicActuator*> actuators = sixBarModel::find<tgBasicActuator>("actuator");
	for (int i = 0; i < actuators.size(); i++) {
		allActuators.push_back(sixBarModel::find<tgBasicActuator>(tgString("actuator num", i))[0]);
	}
	
	// Get the payload for controller
	payload = sixBarModel::find<tgRod>("payload");

	// Notify controllers that setup has finished
	notifySetup();

	// Actually setup the children
	tgModel::setup(world);
}

void sixBarModel::step(double dt)
{
	if (dt <= 0.0) {
		throw std::invalid_argument("dt is not positive");
	}
	else {
	notifyStep(dt);
	tgModel::step(dt);
	}
}

void sixBarModel::onVisit(tgModelVisitor& r)
{
	tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& sixBarModel::getAllActuators() const
{
	return allActuators;
}

const std::vector<tgRod*>& sixBarModel::getAllRods() const 
{
	return allRods;
}

const std::vector<tgRod*>& sixBarModel::getPayload() const 
{
	return payload;
}

const std::vector<btVector3>& sixBarModel::getNormVects() const 
{
	return normalVectors;
}

void sixBarModel::teardown()
{
	notifyTeardown();
	tgModel::teardown();
}

void sixBarModel::addSixBarNodes(tgStructure& s)
{
	/* Starting configuration where the rods are parallel to the principle directions
	 * and the origin is at the center of the tensegrity
	 */
	
	// Calculate the space between two parallel rods based on the rod length from Config
	double rodSpace = (-config.rodLength + sqrt(pow(config.rodLength,2)+4*pow(config.rodLength,2)))/2;

	// Nodes in the x-z plane
	s.addNode(-rodSpace/2, 0, config.rodLength/2); // 0
	s.addNode(-rodSpace/2, 0, -config.rodLength/2); // 1
	s.addNode(rodSpace/2, 0, -config.rodLength/2); // 2
	s.addNode(rodSpace/2, 0, config.rodLength/2); // 3

	// Nodes in the y-z plane
	s.addNode(0, -config.rodLength/2, rodSpace/2); // 4
	s.addNode(0, config.rodLength/2, rodSpace/2); // 5
	s.addNode(0, config.rodLength/2, -rodSpace/2); // 6
	s.addNode(0, -config.rodLength/2, -rodSpace/2); // 7

	// Nodes in the x-y plane
	s.addNode(-config.rodLength/2, -rodSpace/2, 0); // 8
	s.addNode(-config.rodLength/2, rodSpace/2, 0); // 9
	s.addNode(config.rodLength/2, rodSpace/2, 0); // 10
	s.addNode(config.rodLength/2, -rodSpace/2, 0); // 11


	// s.addNode(-rodSpace/2, 0, config.motorLength/2); // 12
	// s.addNode(-rodSpace/2, 0, -config.motorLength/2); // 13
	// s.addNode(rodSpace/2, 0, -config.motorLength/2); //14
	// s.addNode(rodSpace/2, 0, config.motorLength/2); //15

	// s.addNode(0, -config.motorLength/2, rodSpace/2); // 16
	// s.addNode(0, config.motorLength/2, rodSpace/2); // 17
	// s.addNode(0, config.motorLength/2, -rodSpace/2); // 18
	// s.addNode(0, -config.motorLength/2, -rodSpace/2); // 19

	// s.addNode(-config.motorLength/2, -rodSpace/2, 0); // 20
	// s.addNode(-config.motorLength/2, rodSpace/2, 0); // 21
	// s.addNode(config.motorLength/2, rodSpace/2, 0); // 22
	// s.addNode(config.motorLength/2, -rodSpace/2, 0); // 23

}

void sixBarModel::addSixBarRods(tgStructure& s)
{
	// s.addPair(0, 12,  tgString("rod num", 0));
	// s.addPair(12, 13, tgString("motor num", 0));
	// s.addPair(13, 1,  tgString("rod num", 1));

	// s.addPair(3, 15,  tgString("rod num", 2));
	// s.addPair(15, 14, tgString("motor num", 1));
	// s.addPair(14, 2,  tgString("rod num", 3));

	// s.addPair(4, 16,  tgString("rod num", 4));
	// s.addPair(16, 17, tgString("motor num", 2));
	// s.addPair(17, 5,  tgString("rod num", 5));

	// s.addPair(7, 19,  tgString("rod num", 6));
	// s.addPair(19, 18, tgString("motor num", 3));
	// s.addPair(18, 6,  tgString("rod num", 7));

	// s.addPair(8, 20,  tgString("rod num", 8));
	// s.addPair(20, 23, tgString("motor num", 4));
	// s.addPair(23, 11, tgString("rod num", 9));

	// s.addPair(9, 21,  tgString("rod num", 10));
	// s.addPair(21, 22, tgString("motor num", 5));
	// s.addPair(22, 10, tgString("rod num", 11));

	s.addPair(0, 1,  tgString("rod num", 0)); // 0
	s.addPair(3, 2,  tgString("rod num", 1)); // 1
	s.addPair(4, 5,  tgString("rod num", 2)); // 2
	s.addPair(7, 6,  tgString("rod num", 3)); // 3
	s.addPair(8, 11, tgString("rod num", 4)); // 4
	s.addPair(9, 10, tgString("rod num", 5)); // 5
}

void sixBarModel::addSixBarActuators(tgStructure& s)
{
	s.addPair(0, 4,  tgString("actuator num", 0)); // 0
	s.addPair(0, 5,  tgString("actuator num", 1)); // 1
	s.addPair(0, 8,  tgString("actuator num", 2)); // 2
	s.addPair(0, 9,  tgString("actuator num", 3)); // 3

	s.addPair(1, 6,  tgString("actuator num", 4)); // 4
	s.addPair(1, 7,  tgString("actuator num", 5)); // 5
	s.addPair(1, 8,  tgString("actuator num", 6)); // 6
	s.addPair(1, 9,  tgString("actuator num", 7)); // 7

	s.addPair(2, 6,  tgString("actuator num", 8)); // 8
	s.addPair(2, 7,  tgString("actuator num", 9)); // 9
	s.addPair(2, 10, tgString("actuator num", 10)); // 10
	s.addPair(2, 11, tgString("actuator num", 11)); // 11

	s.addPair(3, 4,  tgString("actuator num", 12)); // 12
	s.addPair(3, 5,  tgString("actuator num", 13)); // 13
	s.addPair(3, 10, tgString("actuator num", 14)); // 14
	s.addPair(3, 11, tgString("actuator num", 15)); // 15

	s.addPair(4, 8,  tgString("actuator num", 16)); // 16
	s.addPair(4, 11, tgString("actuator num", 17)); // 17

	s.addPair(5, 9,  tgString("actuator num", 18)); // 18
	s.addPair(5, 10, tgString("actuator num", 19)); // 19

	s.addPair(6, 9,  tgString("actuator num", 20)); // 20
	s.addPair(6, 10, tgString("actuator num", 21)); // 21

	s.addPair(7, 8,  tgString("actuator num", 22)); // 22
	s.addPair(7, 11, tgString("actuator num", 23)); // 23
}

void sixBarModel::addPayloadNodes(tgStructure& s)
{
	double payloadLength = 3;
	s.addNode(0, payloadLength/2, 0); // 12 or 24
	s.addNode(0, -payloadLength/2, 0); //13 0r 25
}

void sixBarModel::addPayloadRods(tgStructure& s)
{
	// s.addPair(24, 25, tgString("payload num", 0));

	s.addPair(12, 13, tgString("payload num", 0));
}

void sixBarModel::addPayloadStrings(tgStructure& s)
{
	// s.addPair(0, 24,  tgString("cable num", 0));
	// s.addPair(4, 24,  tgString("cable num", 1));
	// s.addPair(8, 24,  tgString("cable num", 2));

	// s.addPair(6, 25,  tgString("cable num", 3));
	// s.addPair(10, 25, tgString("cable num", 4));
	// s.addPair(2, 25,  tgString("cable num", 5));

	s.addPair(0, 12,  tgString("cable num", 0));
	s.addPair(4, 12,  tgString("cable num", 1));
	s.addPair(8, 12,  tgString("cable num", 2));

	s.addPair(6, 13,  tgString("cable num", 3));
	s.addPair(10, 13, tgString("cable num", 4));
	s.addPair(2, 13,  tgString("cable num", 5));
}

void sixBarModel::rotateToFace(tgStructure& s, int face)
{
	btVector3 faceNorm = normalVectors[face];
	btVector3 goalDir = btVector3(0, -1, 0);
	double dotProd = faceNorm.dot(goalDir);
	double theta = acos(dotProd);
	btVector3 crossProd = faceNorm.cross(goalDir);

	s.addRotation(btVector3(0,0,0), crossProd, theta);
}
