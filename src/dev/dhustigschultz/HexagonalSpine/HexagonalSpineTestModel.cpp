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
 * @file HexagonalSpineTestModel.cpp
 * @brief Contains the implementation of class HexagonalSpineTestModel
 * @author Lauren Sharo
 * $Id$
 */

// This module
#include "../HexagonalSpine/HexagonalSpineTestModel.h"

#include "core/terrain/tgBulletGround.h"
#include "core/terrain/tgCraterGround.h"
#include "core/tgCast.h"
#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "tgcreator/tgKinematicActuatorInfo.h"
#include "tgcreator/tgKinematicContactCableInfo.h"
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

//#define USE_KINEMATIC

HexagonalSpineTestModel::HexagonalSpineTestModel(size_t segmentsLeft,
		size_t segmentsRight) :
		tgModel(), m_segmentsLeft(segmentsLeft), m_segmentsRight(segmentsRight)
{
}

// Global Variable Declarations

// Creates a Quadruped structure
bool PELVIS = true;
// The height of each leg
const double legSegmentHeight = 4.0;
// Starting height of the structure
const double startHeight = legSegmentHeight + 1.0;

const double density = 5.0;				// The total density of the structure
const double stiffness = 2500.0;     	// For non-passive actuators
const double baseStiffness = 10000.0; 	// For passive actuators
const double damping = 10.0;			//
const double pretension = 600;			//
const bool   history = true;			// History is used for calculating fitness in the controller
const double maxTens = 7000.0;			// For spring cable actuators
const double maxSpeed = 12.0;			// For spring cable actuators
const double mRad = 1.0;			// For kinematic actuators
const double motorFriction = 10.0;		// For kinematic actuators
const double motorInertia = 1.0;		// For kinematic actuators
const bool backDrivable = false;		// For kinematic actuators

const double structureRadius = 1.0;		// Radius of the Hexagonal structures
const double rodRadius = 0.075;			// Radius of the rods in the structure
const double childOffset = 1.5;			// Offset for child structure placement
const double rotation = M_PI / 3.0;		// Node offset for Hexagonal rings
const double doubleRotation = 2.0 * rotation;// Node offset for Triangular rings
const double circumference = M_PI * (rodRadius * rodRadius);	// Rod Circumference
const btVector3 leftOffset(0, 0, -1.2);		// Left structure offset
const btVector3 rightOffset(0, 0, 1.2);	// Right structure offset

const double xOffset = 0.0;

const btVector3 startPoint(xOffset, startHeight, 0.0);	// Center structure offset
const btVector3 leftHexStart(xOffset, startHeight, childOffset); // Left structure start point
const btVector3 rightHexStart(xOffset, startHeight, -childOffset); // Right structure start point

/**
 * Anonymous namespace for helper functions
 */
namespace
{

// Adds the Hexagonal Ring Nodes of all Major Structures
void addHexNodes(tgStructure& structure, double z)
{
	for (int i = 0; i != 6; i++)
	{
		structure.addNode(structureRadius * sin(i * M_PI / 3.0),
				structureRadius * cos(i * M_PI / 3.0), z);
	}
}

// Adds the Nodes of Leg Structures
void addLegNodes(tgStructure& structure, double start, double end,
		double pelvisCenter, double offset, double ringRadius,
		double ringHeight, double twist)
{
	structure.addNode(offset, start, pelvisCenter);
	structure.addNode(offset, end, pelvisCenter);
	for (int i = 0; i < 3; i++)
	{
		structure.addNode(ringRadius * sin(i * doubleRotation - twist) + offset,
				ringHeight,
				ringRadius * cos(i * doubleRotation - twist) + pelvisCenter);
	}

	structure.addNode(-offset, start, pelvisCenter);
	structure.addNode(-offset, end, pelvisCenter);
	for (int i = 0; i < 3; i++)
	{
		structure.addNode(ringRadius * sin(i * doubleRotation + twist) - offset,
				ringHeight,
				ringRadius * cos(i * doubleRotation + twist) + pelvisCenter);
	}
}

// Adds all Nodes
void addAllNodes(tgStructure& hexPrime, tgStructure& leftHex,
		tgStructure& rightHex, tgStructure& leftPelvis,
		tgStructure& rightPelvis)
{
	// The structures are built horizontally along the z-axis
	// beginning with the point of leftHex at 0.0
	const double leftHex_end = 2.0;			// leftHex Hexagonal Ring
	const double centerHex_start = 1.5;		// centerHex/left point
	const double centerHex_midStart = 3.0;	// centerHex/left Hexagonal Ring
	const double centerHex_midEnd = 4.0;	// centerHex/right Hexagonal Ring
	const double centerHex_end = 5.5;		// centerHex/right point
	const double rightHex_end = 5.0;		// rightHex Hexagonal Ring
	const double rightHex_start = 7.0;		// rightHex point
	
	// If PELVIS is true:
	const double leftPelvis_center = 1.0;	// Center of leftPelvis
	const double rightPelvis_center = 6.0;	// Center of rightPelvis
	const double pelvicOffset = 2.5;		// x-Offset of Pelvic point
	const double offset = 1.5;				// x-Offset of Leg
	const double leg_startHeight = 0.5;		// Leg start height
	const double leg_endHeight = 0.5 - legSegmentHeight;// Leg end height
	const double leg_ringHeight = leg_endHeight / 2;	// Leg Ring height
	const double leg_ringRadius = 0.8;		// Leg Ring radius
	const double footHeight = leg_endHeight + 0.5;	// Foot start height
	const double toeHeight = leg_endHeight - 0.5;	// Foot end height
	const double twist = M_PI / 6;			// Leg rotational offset

	// Center structure Nodes 0-13
	hexPrime.addNode(0, 0, centerHex_start);
	addHexNodes(hexPrime, centerHex_midStart);
	addHexNodes(hexPrime, centerHex_midEnd);
	hexPrime.addNode(0, 0, centerHex_end);

	// Left structure Nodes 0-6
	leftHex.addNode(0, 0, 0);
	addHexNodes(leftHex, leftHex_end);

	// Right structure Nodes 0-6
	rightHex.addNode(0, 0, rightHex_start);
	addHexNodes(rightHex, rightHex_end);

	// Left Pelvic structure Nodes 0-8 (Pelvis)
	leftPelvis.addNode(0, 0, 0);
	addHexNodes(leftPelvis, leftHex_end);
	leftPelvis.addNode(pelvicOffset, 0, leftPelvis_center);
	leftPelvis.addNode(-pelvicOffset, 0, leftPelvis_center);

	// Left Pelvic structure Nodes 9-18 (Legs)
	addLegNodes(leftPelvis, leg_startHeight, leg_endHeight, leftPelvis_center,
			offset, leg_ringRadius, leg_ringHeight, twist);

	// Left Pelvic structure Nodes 19-26 (Feet)
	leftPelvis.addNode(offset, footHeight, leftPelvis_center);
	for (int i = 0; i < 3; i++)
	{
		leftPelvis.addNode(
				leg_ringRadius*1.5 * sin(i * doubleRotation + M_PI) + offset,
				toeHeight,
				leg_ringRadius*1.5 * cos(i * doubleRotation + M_PI)
						+ leftPelvis_center);
	}
	leftPelvis.addNode(-offset, footHeight, leftPelvis_center);
	for (int i = 0; i < 3; i++)
	{
		leftPelvis.addNode(
				leg_ringRadius*1.5 * sin(i * doubleRotation + M_PI) - offset,
				toeHeight,
				leg_ringRadius*1.5 * cos(i * doubleRotation + M_PI)
						+ leftPelvis_center);
	}

	// Right Pelvic structure Nodes 0-8 (Pelvis)
	rightPelvis.addNode(0, 0, rightHex_start);
	addHexNodes(rightPelvis, rightHex_end);
	rightPelvis.addNode(pelvicOffset, 0, rightPelvis_center);
	rightPelvis.addNode(-pelvicOffset, 0, rightPelvis_center);

	// Right Pelvic structure Nodes 9-18 (Legs)
	addLegNodes(rightPelvis, leg_startHeight, leg_endHeight, rightPelvis_center,
			offset, leg_ringRadius, leg_ringHeight, twist);

	// Right Pelvic structure Nodes 19-26 (Feet)
	rightPelvis.addNode(offset, footHeight, rightPelvis_center);
	for (int i = 0; i < 3; i++)
	{
		rightPelvis.addNode(-leg_ringRadius*1.5 * sin(i * doubleRotation) + offset,
				toeHeight,
				leg_ringRadius*1.5 * cos(i * doubleRotation) + rightPelvis_center);
	}
	rightPelvis.addNode(-offset, footHeight, rightPelvis_center);
	for (int i = 0; i < 3; i++)
	{
		rightPelvis.addNode(-leg_ringRadius*1.5 * sin(i * doubleRotation) - offset,
				toeHeight,
				leg_ringRadius*1.5 * cos(i * doubleRotation) + rightPelvis_center);
	}
}

// Creates all Left and Right Segment Rods
void addHexPairs(tgStructure& structure, std::string j)
{
	for (int i = 1; i < 7; i++)
	{
		structure.addPair(i, 0, j + " structure rod");
	}
	for (int i = 1; i < 6; i++)
	{
		structure.addPair(i, i + 1, j + " structure ring rod");
	}
	structure.addPair(1, 6, j + " structure ring rod");
}

// Creates all Pelvic and Leg Rods
void addLegPairs(tgStructure& structure, std::string j)
{
	// Pelvis
	addHexPairs(structure, j);
	structure.addPair(7, 0, j + " pelvic rod");
	structure.addPair(7, 2, j + " pelvic rod");
	structure.addPair(7, 4, j + " pelvic rod");
	structure.addPair(8, 0, j + " pelvic rod");
	structure.addPair(8, 4, j + " pelvic rod");
	structure.addPair(8, 6, j + " pelvic rod");

	// Legs
	for (int i = 11; i <= 13; i++)
	{
		structure.addPair(9, i, "top leg to ring rod");
		structure.addPair(10, i, "bottom leg to ring rod");
	}
	structure.addPair(11, 12, "leg ring rod");
	structure.addPair(12, 13, "leg ring rod");
	structure.addPair(13, 11, "leg ring rod");
	for (int i = 16; i <= 18; i++)
	{
		structure.addPair(14, i, "top leg to ring rod");
		structure.addPair(15, i, "bottom leg to ring rod");
	}
	structure.addPair(16, 17, "leg ring rod");
	structure.addPair(17, 18, "leg ring rod");
	structure.addPair(18, 16, "leg ring rod");
	// Feet
	for (int i = 20; i <= 22; i++)
	{
		structure.addPair(19, i, "foot rod");
	}
	structure.addPair(20, 21, "foot rod");
	structure.addPair(21, 22, "foot rod");
	structure.addPair(22, 20, "foot rod");
	for (int i = 24; i <= 26; i++)
	{
		structure.addPair(23, i, "foot rod");
	}
	structure.addPair(24, 25, "foot rod");
	structure.addPair(25, 26, "foot rod");
	structure.addPair(26, 24, "foot rod");
}

// Creates all the Rods
void addAllPairs(tgStructure& hexPrime, tgStructure& leftHex,
		tgStructure& rightHex, tgStructure& leftPelvis,
		tgStructure& rightPelvis)
{
	// Center structure
	for (int i = 1; i < 7; i++)
	{
		hexPrime.addPair(i, 0, "center/left structure rod");
	}
	for (int i = 1; i < 6; i++)
	{
		hexPrime.addPair(i, i + 1, "center/left ring rod");
	}
	hexPrime.addPair(1, 6, "center/left ring rod");
	for (int i = 1; i < 7; i++)
	{
		hexPrime.addPair(i, i + 6, "center structure rod");
	}
	for (int i = 7; i < 12; i++)
	{
		hexPrime.addPair(i, i + 1, "center/right ring rod");
	}
	hexPrime.addPair(7, 12, "center/right ring rod");
	for (int i = 7; i < 13; i++)
	{
		hexPrime.addPair(i, 13, "center/right structure rod");
	}

	// Left structure
	addHexPairs(leftHex, "left");

	// Right structure
	addHexPairs(rightHex, "right");

	// Left Pelvis and Legs
	addLegPairs(leftPelvis, "left");

	// Right Pelvis and Legs
	addLegPairs(rightPelvis, "right");
}

// Builds a Single Segment
void addSegment(tgStructure& spine, const tgStructure& structure,
		btVector3 offset, int i)
{
	tgStructure* const t = new tgStructure(structure);
	t->addTags(tgString("segment", i));
	t->move(offset);
	spine.addChild(t);
}

// Builds all Segments
void addAllSegments(tgStructure& spine, const tgStructure& hexPrime,
		const tgStructure& leftHex, const tgStructure& rightHex,
		const tgStructure& leftPelvis, const tgStructure& rightPelvis,
		const int segLeft, const int segRight)
{
	const int totalSegments = segLeft + segRight;

	// Creates the Center Start Segment
	tgStructure* const t = new tgStructure(hexPrime);
	t->addTags(tgString("segment", 0));
	spine.addChild(t);

	// Creates all remaining Segments
	for (int i = 1; i <= totalSegments; ++i)
	{
		if (!PELVIS) // If PELVIS is false, create normal spine
		{
			if (i <= segLeft)
			{	// Left Segments
				addSegment(spine, leftHex, i * leftOffset, i);
			}
			else
			{	// Right Segments
				addSegment(spine, rightHex, (i - segLeft) * rightOffset, i);
			}
		}
		else // If PELVIS is true, add pelvic structures to each end of the spine
		{
			if (i < segLeft)
			{	// Left Segments
				addSegment(spine, leftHex, i * leftOffset, i);
			}
			else if (i == segLeft)
			{	// Left Pelvis
				addSegment(spine, leftPelvis, i * leftOffset, i);
			}
			else if (i < totalSegments)
			{	// Right Segments
				addSegment(spine, rightHex, (i - segLeft) * rightOffset, i);
			}
			else if (i == totalSegments)
			{	// Right Pelvis
				addSegment(spine, rightPelvis, (i - segLeft) * rightOffset, i);
			}
		}
	}
}

// Attaches all Center Hex Actuators
void addCenterActuators(tgStructure& spine, std::vector<tgStructure*> children,
		std::string j, int i)
{
	// Retrieves the Nodes
	tgNodes prime = children[0]->getNodes();
	tgNodes S1 = children[i + 1]->getNodes();
	int node;

	if (j == "left")
	{	// Attaches the first Left-hand Segment onto the Center structure
		node = 0;
		// Actuators with normal stiffness
		spine.addPair(prime[1], S1[6], "center/left actuator");
		spine.addPair(prime[1], S1[2], "center/left actuator");
		spine.addPair(prime[3], S1[2], "center/left actuator");
		spine.addPair(prime[3], S1[4], "center/left actuator");
		spine.addPair(prime[5], S1[4], "center/left actuator");
		spine.addPair(prime[5], S1[6], "center/left actuator");
	}
	else
	{	// Attaches the first Right-hand Segment onto the Center structure
		node = 13;
		// Actuators with normal stiffness
		spine.addPair(prime[8], S1[1], "center/right actuator");
		spine.addPair(prime[8], S1[3], "center/right actuator");
		spine.addPair(prime[10], S1[3], "center/right actuator");
		spine.addPair(prime[10], S1[5], "center/right actuator");
		spine.addPair(prime[12], S1[5], "center/right actuator");
		spine.addPair(prime[12], S1[1], "center/right actuator");
	}
	// Actuators with high stiffness
	for (int i = 1; i < 7; i++)
	{
		spine.addPair(prime[node], S1[i], "center/" + j + " baseActuator");
	}
}

// Attaches all Segment Actuators
void addSegmentActuators(tgStructure& spine, std::vector<tgStructure*> children,
		std::string j, int i)
{
	// Retrieves the Nodes
	tgNodes S1 = children[i]->getNodes();
	tgNodes S2 = children[i + 1]->getNodes();

	// Actuators with high stiffness
	for (int k = 1; k < 7; k++)
	{
		spine.addPair(S1[0], S2[k], j + " hex baseActuator");
	}
	// Accounts for the Actuator offset
	// between the Left and Right Segments
	if (j == "right")
	{
		S1 = children[i + 1]->getNodes();
		S2 = children[i]->getNodes();
	}

	// Actuators with normal stiffness
	spine.addPair(S1[1], S2[6], j + " hex actuator");
	spine.addPair(S1[1], S2[2], j + " hex actuator");
	spine.addPair(S1[3], S2[2], j + " hex actuator");
	spine.addPair(S1[3], S2[4], j + " hex actuator");
	spine.addPair(S1[5], S2[4], j + " hex actuator");
	spine.addPair(S1[5], S2[6], j + " hex actuator");

}

// Attaches all Leg and Foot Actuators
void addLegActuators(tgStructure& spine, std::vector<tgStructure*> children,
		int i)
{
	// Retrieves the Nodes
	tgNodes leg = children[i]->getNodes();

	// Actuators connecting the first Leg to the Pelvis
	spine.addPair(leg[9], leg[0], "top leg baseActuator");
	spine.addPair(leg[9], leg[2], "top leg baseActuator");
	spine.addPair(leg[9], leg[7], "top leg baseActuator");
	spine.addPair(leg[11], leg[4], "bottom leg actuator");
	spine.addPair(leg[11], leg[7], "bottom leg actuator");
	spine.addPair(leg[12], leg[7], "bottom leg actuator");
	spine.addPair(leg[12], leg[0], "bottom leg actuator");
	spine.addPair(leg[13], leg[0], "bottom leg actuator");
	spine.addPair(leg[13], leg[4], "bottom leg actuator");
	// Actuators connecting the first Foot to the first Leg
	spine.addPair(leg[19], leg[10], "leg to foot baseActuator");
	spine.addPair(leg[20], leg[12], "leg to foot actuator");
	spine.addPair(leg[20], leg[13], "leg to foot actuator");
	spine.addPair(leg[21], leg[13], "leg to foot actuator");
	spine.addPair(leg[21], leg[11], "leg to foot actuator");
	spine.addPair(leg[22], leg[11], "leg to foot actuator");
	spine.addPair(leg[22], leg[12], "leg to foot actuator");
	spine.addPair(leg[10], leg[20], "leg to foot actuator");
	spine.addPair(leg[10], leg[21], "leg to foot actuator");
	spine.addPair(leg[10], leg[22], "leg to foot actuator");

	// Actuators connecting the second Leg to the Pelvis
	spine.addPair(leg[14], leg[0], "top leg baseActuator");
	spine.addPair(leg[14], leg[6], "top leg baseActuator");
	spine.addPair(leg[14], leg[8], "top leg baseActuator");
	spine.addPair(leg[16], leg[8], "bottom leg actuator");
	spine.addPair(leg[16], leg[4], "bottom leg actuator");
	spine.addPair(leg[17], leg[4], "bottom leg actuator");
	spine.addPair(leg[17], leg[0], "bottom leg actuator");
	spine.addPair(leg[18], leg[0], "bottom leg actuator");
	spine.addPair(leg[18], leg[8], "bottom leg actuator");
	// Actuators connecting the second Foot to the second Leg
	spine.addPair(leg[23], leg[15], "leg to foot baseActuator");
	spine.addPair(leg[24], leg[17], "leg to foot actuator");
	spine.addPair(leg[24], leg[18], "leg to foot actuator");
	spine.addPair(leg[25], leg[18], "leg to foot actuator");
	spine.addPair(leg[25], leg[16], "leg to foot actuator");
	spine.addPair(leg[26], leg[16], "leg to foot actuator");
	spine.addPair(leg[26], leg[17], "leg to foot actuator");
	spine.addPair(leg[15], leg[24], "leg to foot actuator");
	spine.addPair(leg[15], leg[25], "leg to foot actuator");
	spine.addPair(leg[15], leg[26], "leg to foot actuator");
}

// Adds all the Actuators
void addAllActuators(tgStructure& spine, int segmentsLeft, int segmentsRight)
{
	// Creates a vector containing all the Segments
	const std::vector<tgStructure*> children = spine.getChildren();
	int totalSegments = children.size();

	// Actuators linking the Center Hex to the first child
	// If there are Left-hand Segments, attach the first Segment onto the Left side
	if (segmentsLeft >= 1)
	{
		addCenterActuators(spine, children, "left", 0);
	}
	// If there are no Left-hand Segments and there are Right-hand Segments,
	// attach the first Segment on the Right side
	else if (segmentsRight >= 1)
	{
		addCenterActuators(spine, children, "right", 0);
	}
	for (int i = 1; i <= totalSegments; i++)
	{
		// Attach all remaining Left-hand Segments
		if (i < segmentsLeft)
		{
			addSegmentActuators(spine, children, "left", i);
		}
		// If the last Left-hand Segment has been reached:
		else if (i == segmentsLeft)
		{
			// If PELVIS is true, attach all Leg and Foot Actuators
			if (PELVIS)
			{
				addLegActuators(spine, children, i);
			}
			// If there are Right-hand Segments, attach the first
			// to the right side of the Center structure
			if (segmentsRight >= 1)
			{
				addCenterActuators(spine, children, "right", i);
			}
		}
		// Attach all remaining Right-hand Segments
		else if (i < totalSegments - 1)
		{
			addSegmentActuators(spine, children, "right", i);
		}
		// If the last Right-hand Segment has been reached and
		// PELVIS is true, attach all Leg and Foot Actuators
		if (i == totalSegments - 1 && PELVIS)
		{
			addLegActuators(spine, children, i);
		}
	}
}

}	// namespace

void HexagonalSpineTestModel::setup(tgWorld& world)
{

	// Create the Structures
	tgStructure spine, hexPrime, leftHex, rightHex, leftPelvis, rightPelvis;
	// Add all Nodes and Rods
	addAllNodes(hexPrime, leftHex, rightHex, leftPelvis, rightPelvis);
	addAllPairs(hexPrime, leftHex, rightHex, leftPelvis, rightPelvis);

	// Move the Structures to their starting points
	hexPrime.move(startPoint);
	leftHex.move(leftHexStart);
	rightHex.move(rightHexStart);
	leftPelvis.move(leftHexStart);
	rightPelvis.move(rightHexStart);

	// Create our Spine Segments and Attach all Actuators
	addAllSegments(spine, hexPrime, leftHex, rightHex, leftPelvis, rightPelvis,
			m_segmentsLeft, m_segmentsRight);
	addAllActuators(spine, m_segmentsLeft, m_segmentsRight);

	// Create the build spec that uses tags to turn the structure into a real model
	tgBuildSpec spec;

	// Rod configuration
	const tgRod::Config rodConfig(rodRadius, density);

	// Actuator Configurations
	// Modified by Dawn to use either kinematic actuators or spring cable actuators, both with optional passive settings. 
	#ifdef USE_KINEMATIC


	    tgKinematicActuator::Config muscleConfig(stiffness, damping, pretension, 
						     mRad, motorFriction, motorInertia, 
						     backDrivable, history, maxTens, maxSpeed);
	    tgKinematicActuator::Config baseActuatorConfig(baseStiffness, damping, pretension, 
							   mRad, motorFriction, motorInertia, 
							   backDrivable, history, maxTens, maxSpeed);	    

	#else
	    tgSpringCableActuator::Config muscleConfig(stiffness, damping, pretension, 
						       history, maxTens, maxSpeed);
	    tgSpringCableActuator::Config baseActuatorConfig(baseStiffness, damping,
							     pretension, history, maxTens, maxSpeed);
	#endif

	// Add all Configurations
	spec.addBuilder("rod", new tgRodInfo(rodConfig));

	#ifdef USE_KINEMATIC	
	    spec.addBuilder("actuator", new tgKinematicContactCableInfo(muscleConfig));
	    spec.addBuilder("baseActuator", new tgKinematicContactCableInfo(baseActuatorConfig));
	#else
	    spec.addBuilder("actuator", new tgBasicActuatorInfo(muscleConfig));
	    spec.addBuilder("baseActuator", new tgBasicActuatorInfo(baseActuatorConfig));
	#endif

	// Rotate the spine
	const btVector3 point(0, 0, 0);
	const btVector3 axis(0, 1, 0);
	spine.addRotation(point, axis, M_PI / 2.0);

	// Build and place structure into the World
	tgStructureInfo structureInfo(spine, spec);
	structureInfo.buildInto(*this, world);

	// Set up the world
	tgModel::setup(world);
}

// Retrieves the Mass of the entire structure
double HexagonalSpineTestModel::getMass()
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

	// Prints the total number of Rods to the command line:
	std::cout << "Number of Rods: " << rods.size() << std::endl;
	// Prints the total mass of the structure to the command line:
	std::cout << "Mass: " << totalMass << " kg" << std::endl;
	return totalMass;
}

// Notifies controllers of the step
void HexagonalSpineTestModel::step(double dt)
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
HexagonalSpineTestModel::getActuators(const std::string& key) const
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
