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
 * @file DOF_JointTestModel.cpp
 * @brief Contains the implementation of class DOF_JointTestModel
 * @author Lauren Sharo
 * $Id$
 */

// This module
#include "DOF_JointTestModel.h"
// This library
#include "core/tgCast.h"
#include "core/tgBasicActuator.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <iostream>
#include <stdexcept>

DOF_JointTestModel::DOF_JointTestModel() :
		tgModel()
{
}

// Global Variable Declarations
const double ringRadius = 15.0;				// Fixed Ring Radius
const double armRadius = 4.78;				// Arm Ring Radius
const double armStart_z = 0.8827;			// Arm start position
const double armRing_z = 16.52 + armStart_z;// Arm Ring position
const double armEnd_z = 35.0 + armRing_z;	// Arm end position
const double ringRotation = M_PI / 12;

//Rod and Ring specifications
const double rodDensity = .00311;			// Arm Density
const double ringDensity = 0.0;				// Fixed Ring Density
const double rodRadius = 0.5;				// Rod Radius
const double rodCircumference = M_PI * (rodRadius * rodRadius); // Rod Circumference

//Actuator specifications
const double stiffness = 2000.0;			// Stiffness for non-passive Actuators
const double baseStiffness = 200000.0;		// Stiffness for passive Actuators
const double damping = 10.0;				//

//Load specifications
const double loadMass = 0.5;				// Load Mass
const double loadRadius = 1.0;				// Load Radius
const double loadHeight = 2.0;				// Load Height
const double loadCircumference = M_PI * (loadRadius * loadRadius); // Load Circumference
const double loadVolume = loadCircumference * loadHeight; // Load Volume
const double loadDensity = loadMass / loadVolume; // Load Density; dependent on loadMass

/**
 * Anonymous namespace for helper functions
 */
namespace
{
// Add all Nodes in the structure
void addAllNodes(tgStructure& joint)
{
	// Base Structure
	for (int i = 0; i < 24; i++)
	{
		joint.addNode(ringRadius * sin(i * ringRotation),
				ringRadius * cos(i * ringRotation), 0);
	}

	// Arm
	joint.addNode(0, 0, armStart_z);
	joint.addNode(armRadius * sin(M_PI / 3.0), armRadius * cos(M_PI / 3.0),
			armRing_z);
	joint.addNode(armRadius * sin(M_PI), armRadius * cos(M_PI), armRing_z);
	joint.addNode(armRadius * sin(5 * M_PI / 3.0), armRadius * cos(5 * M_PI / 3.0),
			armRing_z);
	joint.addNode(0, 0, armEnd_z);

	// Load
	joint.addNode(0, 1, armEnd_z);
	joint.addNode(0, -1, armEnd_z);
}

// Create all Rods in the structure
void addPairs(tgStructure& joint)
{
	// Fixed Ring Rods
	for (int i = 0; i < 23; i++)
	{
		joint.addPair(i, i + 1, "stationary ring");
	}
	joint.addPair(0, 23, "stationary ring");
	// Arm Rods
	joint.addPair(24, 25, "start arm rod");
	joint.addPair(24, 26, "start arm rod");
	joint.addPair(24, 27, "start arm rod");
	joint.addPair(25, 26, "middle arm ring rod");
	joint.addPair(26, 27, "middle arm ring rod");
	joint.addPair(27, 25, "middle arm ring rod");
	joint.addPair(25, 28, "end arm rod");
	joint.addPair(27, 28, "end arm rod");
	joint.addPair(26, 28, "end arm rod");
	// Load Rods
	joint.addPair(28, 29, "load");
	joint.addPair(28, 30, "load");
}

// Add Actuators to connect the Arm to the Fixed Ring
void addActuators(tgStructure& joint)
{
	// Connects the Start Node of the Arm to the Fixed Ring
	joint.addPair(0, 24, "stiffActuator");
	joint.addPair(8, 24, "stiffActuator");
	joint.addPair(16, 24, "stiffActuator");
	// Connects the Ring Nodes of the Arm to the Fixed Ring
	joint.addPair(0, 25, "actuator");
	joint.addPair(0, 27, "actuator");
	joint.addPair(8, 25, "actuator");
	joint.addPair(8, 26, "actuator");
	joint.addPair(16, 26, "actuator");
	joint.addPair(16, 27, "actuator");
}

/**
 Methods not currently in use

 void mapActuators(DOF_JointTestModel::ActuatorMap& actuatorMap, tgModel& model)
 {
 // Note that tags don't need to match exactly, we could create
 // supersets if we wanted to
 actuatorMap["inner left"] = model.find<tgBasicActuator>(
 "inner left actuator");
 actuatorMap["inner right"] = model.find<tgBasicActuator>(
 "inner right actuator");
 actuatorMap["inner top"] = model.find<tgBasicActuator>(
 "inner top actuator");
 actuatorMap["outer left"] = model.find<tgBasicActuator>(
 "outer left actuator");
 actuatorMap["outer right"] = model.find<tgBasicActuator>(
 "outer right actuator");
 actuatorMap["outer top"] = model.find<tgBasicActuator>(
 "outer top actuator");
 }

 void trace(const tgStructureInfo& structureInfo, tgModel& model)
 {
 std::cout << "StructureInfo:" << std::endl << structureInfo << std::endl
 << "Model: " << std::endl << model << std::endl;
 }
 */

} // namespace

void DOF_JointTestModel::setup(tgWorld& world)
{
	// Create the joint
	tgStructure joint;
	// Add all Nodes, Pairs, and Actuators
	addAllNodes(joint);
	addPairs(joint);
	addActuators(joint);

	// Move the joint
	joint.move(btVector3(0.0, 20.0, 0.0));

	// Create the build spec that uses tags to turn the structure into a real model
	tgBuildSpec spec;

	// Rod Configuration
	const tgRod::Config rodConfig(rodRadius, rodDensity);
	// Ring Configuration
	const tgRod::Config ringConfig(rodRadius, ringDensity);
	// Load Configuration
	const tgRod::Config loadConfig(loadRadius, loadDensity);
	spec.addBuilder("load", new tgRodInfo(loadConfig));
	// Actuator Configurations
	tgBasicActuator::Config actuatorConfig(stiffness, damping);
	tgBasicActuator::Config stiffActuatorConfig(baseStiffness, damping);

	// Add all configurations to the build spec
	spec.addBuilder("rod", new tgRodInfo(rodConfig));
	spec.addBuilder("ring", new tgRodInfo(ringConfig));
	spec.addBuilder("actuator", new tgBasicActuatorInfo(actuatorConfig));
	spec.addBuilder("stiffActuator",
			new tgBasicActuatorInfo(stiffActuatorConfig));

	// Create the structureInfo
	tgStructureInfo structureInfo(joint, spec);
	// Use the structureInfo to build the structure
	structureInfo.buildInto(*this, world);

	// Set up the structure
	tgModel::setup(world);

//	// We could now use tgCast::filter or similar to pull out the models (e.g. actuators)
//	// that we want to control.
//	allActuators = tgCast::filter<tgModel, tgBasicActuator>(getDescendants());
//	mapActuators(actuatorMap, *this);
//
//	trace(structureInfo, *this);
}

// Retrieves the Mass of the entire structure
double DOF_JointTestModel::getMass()
{
	// Creates a vector containing all the Rods in the structure
	std::vector<tgRod*> rods = find<tgRod>("rod");
	// Asserts the vector is not empty
	assert(!rods.empty());

	double totalMass = 0;
	// Cycles through all the Rods in the vector
	for (std::size_t i = 0; i < rods.size(); i++)
	{
		const tgRod* const rod = rods[i];
		assert(rod != NULL);
		// Calculates the mass of the Rod
		const double volume = rod->length() * rodCircumference;
		const double rodMass = rodDensity * volume;
		// Adds the Rod's mass to the total mass of the structure
		totalMass += rodMass;
	}

	// Prints the masses of the Arm and Load to the command line:
	std::cout << "Arm Mass: " << totalMass << std::endl;
	std::cout << "Load Mass: " << loadMass << std::endl;
	return totalMass;
}

// Notifies controllers of the step
void DOF_JointTestModel::step(double dt)
{
	if (dt < 0.0)
	{
		throw std::invalid_argument("dt is not positive");
	}
	else
	{
		// Notify observers (controllers) of the step so that they can take action
		notifyStep(dt);
		// Step any children
		tgModel::step(dt);
	}
}

// Retrieves All the Actuators
const std::vector<tgBasicActuator*>& DOF_JointTestModel::getActuators(
		const std::string& key) const
{
	const ActuatorMap::const_iterator it = actuatorMap.find(key);
	if (it == actuatorMap.end())
	{
		throw std::invalid_argument(
				"Key '" + key + "' not found in actuator map");
	}
	else
	{
		return it->second;
	}
}

