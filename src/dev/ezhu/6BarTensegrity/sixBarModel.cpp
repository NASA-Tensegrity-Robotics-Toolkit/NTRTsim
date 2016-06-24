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

namespace 
{
	/**
   * Configuration parameters so they're easily accessable.
   * All parameters must be positive.
   */

   double sf = 1;

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
   	} config =
   		{
   			0.688/pow(sf,3),    // density (kg / length^3)
	        0.31*sf,     // radius (length)
	        1615.0,   // stiffness (kg / sec^2) was 1500
	        200.0,    // damping (kg / sec)
	        16.84*sf,     // rodLength (length)
	        0.99,      // friction (unitless)
	        0.01,     // rollFriction (unitless)
	        0.0,      // restitution (?)
	        3000.0*sf,        // pretension -> set to 4 * 613, the previous value of the rest length controller
	        0,         // History logging (boolean)
	        100000*sf,   // maxTension
	        10000*sf,    // targetVelocity   
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
	// Define the configurations of the rods and strings
  	// Note that pretension is defined for this string
	const tgRod::Config sixBarRodConfig(config.radius, config.density, config.friction,
		config.rollFriction, config.restitution);
	const tgRod::Config PayloadRodConfig(config.radius, config.density, config.friction,
		config.rollFriction, config.restitution);

	tgBasicActuator::Config actuatorConfig(config.stiffness, config.damping, config.pretension,
		config.hist, config.maxTension, config.targetVelocity);
	tgBasicActuator::Config staticCableConfig(config.stiffness, config.damping, config.pretension,
		config.hist, config.maxTension, config.targetVelocity);

	// Create a structure that will hold the details of this model
	tgStructure s;

	// Add in the tensegrity structure
	addSixBar(s);

	// Add in the payload
	//addPayload(s);

	// Move the structure
	//s.addRotation(btVector3(0,0,0), btVector3(0,0,1), M_PI);
	s.move(btVector3(0,10,0));

	// Create the build spec that uses tags to turn the structure into a real model
	tgBuildSpec spec;
	spec.addBuilder("rod", new tgRodInfo(sixBarRodConfig));
	spec.addBuilder("payload", new tgRodInfo(PayloadRodConfig));
	spec.addBuilder("actuator", new tgBasicActuatorInfo(actuatorConfig));
	spec.addBuilder("cable", new tgBasicActuatorInfo(staticCableConfig));

	// Create the structureInfo
	tgStructureInfo structureInfo(s, spec);
	structureInfo.buildInto(*this, world);

	// Get actuators
	allActuators = getAllActuators();

	// Notify controllers that setup has finished
	notifySetup();

	// Actually setup the children
	tgModel::setup(world);
}

void sixBarModel::step(double dt)
{
	notifyStep(dt);
	tgModel::step(dt);
}

void sixBarModel::onVisit(tgModelVisitor& r)
{
	tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& sixBarModel::getAllActuators() const
{
	return allActuators;
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
	double rodSpace = (-config.rodLength + sqrt(pow(config.rodLength,2)+4*config.rodLength))/2;

	// Nodes in the x-z plane
	s.addNode(-rodSpace/2, 0, -config.rodLength/2); // 0
	s.addNode(-rodSpace/2, 0, config.rodLength/2); // 1
	s.addNode(rodSpace/2, 0, config.rodLength/2); // 2
	s.addNode(rodSpace/2, 0, -config.rodLength/2); // 3

	// Nodes in the y-z plane
	s.addNode(0, -config.rodLength/2, -rodSpace/2); // 4
	s.addNode(0, config.rodLength/2, -rodSpace/2); // 5
	s.addNode(0, config.rodLength/2, rodSpace/2); // 6
	s.addNode(0, -config.rodLength/2, rodSpace/2); // 7

	// Nodes in the x-y plane
	s.addNode(-config.rodLength/2, -rodSpace/2, 0); // 8
	s.addNode(-config.rodLength/2, rodSpace/2, 0); // 9
	s.addNode(config.rodLength/2, rodSpace/2, 0); // 10
	s.addNode(config.rodLength/2, -rodSpace/2, 0); // 11
}

void sixBarModel::addSixBarRods(tgStructure& s)
{
	s.addPair(0, 1, "rod");
	s.addPair(2, 3, "rod");
	s.addPair(4, 5, "rod");
	s.addPair(6, 7, "rod");
	s.addPair(8, 11, "rod");
	s.addPair(9, 10, "rod");
}

void sixBarModel::addSixBarActuators(tgStructure& s)
{
	s.addPair(0, 4, "actuator");
	s.addPair(0, 5, "actuator");
	s.addPair(0, 8, "actuator");
	s.addPair(0, 9, "actuator");

	s.addPair(1, 6, "actuator");
	s.addPair(1, 7, "actuator");
	s.addPair(1, 8, "actuator");
	s.addPair(1, 9, "actuator");

	s.addPair(2, 6, "actuator");
	s.addPair(2, 7, "actuator");
	s.addPair(2, 10, "actuator");
	s.addPair(2, 11, "actuator");

	s.addPair(3, 4, "actuator");
	s.addPair(3, 5, "actuator");
	s.addPair(3, 10, "actuator");
	s.addPair(3, 11, "actuator");

	s.addPair(4, 8, "actuator");
	s.addPair(4, 11, "actuator");

	s.addPair(5, 9, "actuator");
	s.addPair(5, 10, "actuator");

	s.addPair(6, 9, "actuator");
	s.addPair(6, 10, "actuator");

	s.addPair(7, 8, "actuator");
	s.addPair(7, 11, "actuator");
}

void sixBarModel::addPayloadNodes(tgStructure& s)
{
	double payloadLength = 3;
	s.addNode(0, payloadLength/2, 0); // 12
	s.addNode(0, -payloadLength/2, 0); //13
}

void sixBarModel::addPayloadRods(tgStructure& s)
{
	s.addPair(12, 13, "payload");
}

void sixBarModel::addPayloadStrings(tgStructure& s)
{

}
