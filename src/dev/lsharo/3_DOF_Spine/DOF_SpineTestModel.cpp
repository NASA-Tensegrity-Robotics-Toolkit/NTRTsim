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
 * @file DOF_SpineTestModel.cpp
 * @brief Contains the implementation of class DOF_SpineTestModel
 * @author Lauren Sharo
 * $Id$
 */

// This module
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
#include "DOF_SpineTestModel.h"

DOF_SpineTestModel::DOF_SpineTestModel(size_t segments) :
tgModel(),
m_segments(segments)
{
}

// Global Variable Declarations
const double radius = 2.0;				// Radius of each spine segment
const double height = 3.0;				// Height of each spine segment
const double density = 1.5;				// Density of all non-stationary segments
const double fixedDensity = 0.0;		// Density of all stationary segments
const double rodRadius = 0.1;			// Radius of all Rods in the structure
const double stiffness = 10000.0;		// Stiffness of non-passive Actuators
const double baseStiffness = 100000.0;	// Stiffness of passive Actuators
const double damping = 10.0;			//
const double pretension = 600;			//
double rotation = M_PI / 12;				// Node offset for ring Rods
const double circumference = M_PI * (rodRadius * rodRadius);	// Rod circumference


/**
 * Anonymous namespace for helper functions
 */
namespace
{

// Creates all the Nodes
void addAllNodes(tgStructure& tetra)
{
	for (int i = 0; i < 24; i++)
	{
		tetra.addNode(radius * sin(i * rotation), radius * cos(i * rotation), 0);
	}
	tetra.addNode(0, 0, height);

}

// Creates the stationary start segment Rods
void addAllFixedPairs(tgStructure& tetraPrime)
{
	// Ring Rods
	for (int i = 0; i < 23; i++)
	{
		tetraPrime.addPair(i, i + 1, "ring fixed");
	}
	tetraPrime.addPair(0, 23, "ring fixed");
	// Structure Rods
	tetraPrime.addPair(0, 24, "front top fixed");
	tetraPrime.addPair(8, 24, "top fixed");
	tetraPrime.addPair(16, 24, "front fixed");
}

// Creates all non-stationary segment Rods
void addAllPairs(tgStructure& tetra)
{
	// Ring Rods
	for (int i = 0; i < 23; i++)
	{
		tetra.addPair(i, i + 1, "ring rod");
	}
	tetra.addPair(0, 23, "ring rod");
	// Structure Rods
	tetra.addPair(0, 24, "front top rod");
	tetra.addPair(8, 24, "top rod");
	tetra.addPair(16, 24, "front rod");
}

// Adds all Segments to the spine structure
void addAllSegments(tgStructure& spine, const tgStructure& tetraPrime,
		const tgStructure& tetra, size_t segmentCount)
{
	// Used to position and rotate the spine segments
	const btVector3 offset(0, 0, -radius * 1.2);
	const btVector3 axis(0, 0, 1);
	const btVector3 fixedPoint(0.0, 0.0, 0.0);

	for (size_t i = 1; i <= segmentCount; ++i)
	{
		// Creates fixed start segment
		if (i == 1)
		{
			tgStructure* const t = new tgStructure(tetraPrime);
			t->addTags(tgString("segment", i));
			t->move(offset);
			t->addRotation(fixedPoint, axis, (i - 1) * 3.14159 / 3.0);
			// Add a child to the spine
			spine.addChild(t);
		}
		// Creates all remaining segments
		else
		{
			tgStructure* const t = new tgStructure(tetra);
			t->addTags(tgString("segment", i));
			t->move((i) * offset);
			t->addRotation(fixedPoint, axis, (i - 1) * 3.14159 / 3.0);
			// Add a child to the spine
			spine.addChild(t);
		}
	}
}

// Adds the Actuators that connect the segments
void addAllActuators(tgStructure& spine)
{
	// Creates a vector containing all the Segments
	const std::vector<tgStructure*> children = spine.getChildren();
	for (size_t i = 1; i < children.size(); ++i)
	{
		// Retrieves the Nodes
		tgNodes n0 = children[i - 1]->getNodes();
		tgNodes n1 = children[i]->getNodes();

		// Actuators with normal stiffness
		spine.addPair(n0[8], n1[16], "actuator");
		spine.addPair(n0[8], n1[8], "actuator");
		spine.addPair(n0[16], n1[16], "actuator");
		spine.addPair(n0[16], n1[0], "actuator");
		spine.addPair(n0[0], n1[0], "actuator");
		spine.addPair(n0[0], n1[8], "actuator");
		// Actuators with high stiffness
		spine.addPair(n0[0], n1[24], "baseActuator");
		spine.addPair(n0[8], n1[24], "baseActuator");
		spine.addPair(n0[16], n1[24], "baseActuator");
	}
}

/**
 Methods not currently in use
void mapActuators(DOF_SpineTestModel::ActuatorMap& actuatorMap,
		tgModel& model)
{
	// Note that tags don't need to match exactly, we could create
	// supersets if we wanted to
	actuatorMap["inner left"] = model.find<tgBasicActuator>("inner left actuator");
	actuatorMap["inner right"] = model.find<tgBasicActuator>("inner right actuator");
	actuatorMap["inner top"] = model.find<tgBasicActuator>("inner top actuator");
	actuatorMap["outer left"] = model.find<tgBasicActuator>("outer left actuator");
	actuatorMap["outer right"] = model.find<tgBasicActuator>("outer right actuator");
	actuatorMap["outer top"] = model.find<tgBasicActuator>("outer top actuator");
}

void trace(const tgStructureInfo& structureInfo, tgModel& model)
{
        std::cout << "StructureInfo:" << std::endl
        << structureInfo    << std::endl
        << "Model: "        << std::endl
        << model            << std::endl;
}
**/

} // namespace

void DOF_SpineTestModel::setup(tgWorld& world)
{
	// Create the start segment
	tgStructure tetraPrime;
	addAllNodes(tetraPrime);
	addAllFixedPairs(tetraPrime);

	// Create all remaining segments
	tgStructure tetra;
	addAllNodes(tetra);
	addAllPairs(tetra);

	// Create the spine and add all Actuators
	tgStructure spine;
	addAllSegments(spine, tetraPrime, tetra, m_segments);
	addAllActuators(spine);

	// Create the build spec that uses tags to turn the structure into a real model
	tgBuildSpec spec;

	// Create all Rod configurations
	const tgRod::Config fixedConfig(rodRadius, fixedDensity);
	const tgRod::Config rodConfig(rodRadius, density);
	// Create all Actuator configurations
	tgBasicActuator::Config baseActuatorConfig(baseStiffness, damping, pretension);
	tgBasicActuator::Config muscleConfig(stiffness, damping, pretension);

	// Add all configurations to the build spec
	spec.addBuilder("fixed", new tgRodInfo(fixedConfig));
	spec.addBuilder("rod", new tgRodInfo(rodConfig));
	spec.addBuilder("actuator", new tgBasicActuatorInfo(muscleConfig));
	spec.addBuilder("baseActuator", new tgBasicActuatorInfo(baseActuatorConfig));

	// Positions and rotates the spine
	const btVector3 point(0.0, 3.0, 0.0);
	const btVector3 axis(1, 0, 0);
	spine.addRotation(point, axis, 3.1415/2.0);

	// Create your structureInfo
	tgStructureInfo structureInfo(spine, spec);
	// Use the structureInfo to build the structure
	structureInfo.buildInto(*this, world);

	// Set up the model
	tgModel::setup(world);

//	// We could now use tgCast::filter or similar to pull out the models (e.g. actuators)
//	// that we want to control.
//	allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
//	mapActuators(actuatorMap, *this);
//
//	trace(structureInfo, *this);
}

// Retrieves the Mass of the entire structure
double DOF_SpineTestModel::getMass()
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
		const double volume = rod->length() * circumference;
		const double rodMass = density * volume;
		// Adds the Rod's mass to the total mass of the structure
		totalMass += rodMass;
	}

	// Prints the total mass of the structure to the command line:
	std::cout << "Mass: " << totalMass << " kg" << std::endl;
	return totalMass;
}

// Notifies controllers of the step
void DOF_SpineTestModel::step(double dt)
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
const std::vector<tgBasicActuator*>&
DOF_SpineTestModel::getActuators(const std::string& key) const
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

