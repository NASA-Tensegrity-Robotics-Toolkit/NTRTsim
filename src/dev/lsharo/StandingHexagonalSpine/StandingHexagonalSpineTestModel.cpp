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
 * @file StandingHexagonalSpineTestModel.cpp
 * @brief Contains the implementation of class StandingHexagonalSpineTestModel
 * @author Lauren Sharo
 * $Id$
 */

// This module
#include "StandingHexagonalSpineTestModel.h"
#include "core/terrain/tgBulletGround.h"
#include "core/terrain/tgCraterGround.h"
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

/**
 * structureHeight calculation is off
 */

StandingHexagonalSpineTestModel::StandingHexagonalSpineTestModel(
		size_t topSegments, size_t bottomSegments) :
		tgModel(), m_topSegments(topSegments), m_bottomSegments(bottomSegments)
{
}

// Global Variable Declarations

// Creates a biped structure
bool PELVIS = true;
// Creates a biped structure with legs that are divided into two segments
bool KNEES = false;
// The length of each leg segment
const double legSegmentHeight = 4.0;
// If KNEES is true:
//	- Do not set this value under 2.5
//	- Multiply by 2 for total leg height

const double density = 5.0;				// The total density of the structure
const double stiffness = 8000.0;      	// For non-passive actuators
const double baseStiffness = 30000.0; 	// For passive actuators
const double damping = 10.0;			//
const double pretension = 600;			//
const double rodRadius = 0.075;			// Radius of the rods in the structure
const double structureRadius = 1.0;		// Radius of the Hexagonal structures
const double childOffset = 1.5;			// Offset for child structure placement
const double PI = 3.14159;				//
const double rotation = PI / 3.0;		// Node offset for Hexagonal rings
const double doubleRotation = 2.0 * rotation;// Node offset for Triangular rings
const double circumference = PI * (rodRadius * rodRadius);	// Rod Circumference
const btVector3 topOffset(0, 0, -1.2);	// Top structure offset
const btVector3 bottomOffset(0, 0, 1.2);	// Bottom structure offset
const btVector3 startPoint(0.0, 10.0, 0.0);		// Base structure start point
const btVector3 topHexStart(0.0, 10.0, childOffset);// Top structure start point
const btVector3 bottomHexStart(0.0, 10.0, -childOffset);// Bottom structure start point

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
		structure.addNode(structureRadius * sin(i * PI / 3.0),
				structureRadius * cos(i * PI / 3.0), z);
	}
}

// Adds the Triangular Ring Nodes of Leg Structures
void addLegNodes(tgStructure& structure, double legOffset, double ringRadius,
		double ringHeight, double twist)
{
	for (int i = 0; i < 3; i++)
	{
		structure.addNode(
				ringRadius * sin(i * doubleRotation + twist) + legOffset,
				ringRadius * cos(i * doubleRotation + twist), ringHeight);
	}
}

// Adds all Foot Nodes
void addFootNodes(tgStructure& structure, double ringRadius, double legOffset,
		double footHeight, double toeHeight)
{
	// Value that determines how wide the Foot is on each side
	const double toeOffset = 0.5;

	structure.addNode(legOffset, 0, footHeight);
	structure.addNode(
			-ringRadius * sin(0 * doubleRotation) + legOffset - toeOffset,
			ringRadius * cos(0 * doubleRotation) + toeOffset, toeHeight);
	structure.addNode(
			-ringRadius * sin(1 * doubleRotation) + legOffset + toeOffset,
			ringRadius * cos(1 * doubleRotation) - (toeOffset * 2.0),
			toeHeight);
	structure.addNode(
			-ringRadius * sin(2 * doubleRotation) + legOffset
					+ (toeOffset * 1.5), ringRadius * cos(2 * doubleRotation),
			toeHeight);

	structure.addNode(-legOffset, 0, footHeight);
	structure.addNode(
			-ringRadius * sin(0 * doubleRotation) - legOffset + toeOffset,
			ringRadius * cos(0 * doubleRotation) + toeOffset, toeHeight);
	structure.addNode(
			-ringRadius * sin(1 * doubleRotation) - legOffset
					- (toeOffset * 1.5), ringRadius * cos(1 * doubleRotation),
			toeHeight);
	structure.addNode(
			-ringRadius * sin(2 * doubleRotation) - legOffset - toeOffset,
			ringRadius * cos(2 * doubleRotation) - (toeOffset * 2.0),
			toeHeight);
}

// Adds all Nodes
void addAllNodes(tgStructure& hexPrime, tgStructure& topHex,
		tgStructure& bottomHex, tgStructure& pelvicHex)
{
	// The structure is built from top to bottom
	// The tallest point is located at Z-coordinate 0.0 (the tip of topHex)

	// Adjusting to account for nesting in pelvis
	const double leg_Height = legSegmentHeight;

	// Z-coordinates of centerHex, topHex, and bottomHex
	const double topHex_end = 2.0;			// topHex Hexagonal Ring
	const double baseHex_start = 1.5;		// base/top Point
	const double baseHex_midStart = 3.0;	// base/top Hexagonal Ring
	const double baseHex_midEnd = 4.0;		// base/bottom Hexagonal Ring
	const double baseHex_end = 5.5;			// base/bottom Point
	const double bottomHex_end = 5.0;		// bottomHex Hexagonal Ring
	const double bottomHex_start = 7.0;		// bottomHex Point

	// If Pelvis is true:
	// Z-coordinates of the Pelvis, Legs, and Feet
	const double pelvis_start = 6.5;		// pelvicHex Hexagonal Ring
	const double pelvis_end = 7.5;			// pelvicHex Point
	const double legStart = 6.5;
	const double legEnd = legStart + leg_Height;
	const double leg_ringHeight = legStart + (leg_Height / 2);
	const double footHeight = legEnd - 0.5;	// Foot start
	const double toeHeight = legEnd + 0.5;	// Foot end
	// Radius of the Leg Ring
	const double leg_ringRadius = 0.8;
	// Radius of the Foot Ring
	const double footRingRadius = leg_ringRadius * 1.3;
	// Distance between the edge and center of the Pelvis
	const double offset = 1.8;
	// Distance between the Legs and the center of the Pelvis
	const double legOffset = 1.0;
	// Rotational offset for the Legs
	const double twist = (PI / 6) + PI;

	// If KNEES is true:
	// Z-coordinates of the second Leg Segment
	const double bottomLegStart = footHeight;
	const double bottomLegEnd = bottomLegStart + leg_Height;
	// New Z-coordinates for the Feet
	const double foot_Height = bottomLegEnd - 1.5;	// Foot start
	const double toe_Height = bottomLegEnd + 0.5;	// Foot end

	// Prime structure Nodes 0-13
	hexPrime.addNode(0, 0, baseHex_start);
	addHexNodes(hexPrime, baseHex_midStart);
	addHexNodes(hexPrime, baseHex_midEnd);
	hexPrime.addNode(0, 0, baseHex_end);

	// Top structure Nodes 0-6
	topHex.addNode(0, 0, 0);
	addHexNodes(topHex, topHex_end);

	// Bottom structure Nodes 0-6
	bottomHex.addNode(0, 0, bottomHex_start);
	addHexNodes(bottomHex, bottomHex_end);

	// If PELVIS is true, attach Pelvic structure and Legs:
	// Pelvic structure Nodes 0-6 (Hexagonal structure)
	pelvicHex.addNode(0, 0, bottomHex_start);
	addHexNodes(pelvicHex, bottomHex_end);
	// Pelvic structure Nodes 7-13 (Pelvis)
	pelvicHex.addNode(offset, offset / 2, pelvis_start);
	pelvicHex.addNode(offset, -offset / 2, pelvis_start);
	pelvicHex.addNode(-offset, offset / 2, pelvis_start);
	pelvicHex.addNode(-offset, -offset / 2, pelvis_start);
	pelvicHex.addNode(0, offset, pelvis_start);
	pelvicHex.addNode(0, -offset, pelvis_start);
	pelvicHex.addNode(0, 0, pelvis_end);
	// Pelvic structure Nodes 14-23 (Legs)
	pelvicHex.addNode(legOffset, 0, legStart);
	pelvicHex.addNode(legOffset, 0, legEnd);
	addLegNodes(pelvicHex, legOffset, leg_ringRadius, leg_ringHeight, -twist);
	pelvicHex.addNode(-legOffset, 0, legStart);
	pelvicHex.addNode(-legOffset, 0, legEnd);
	addLegNodes(pelvicHex, -legOffset, leg_ringRadius, leg_ringHeight, twist);

	// If KNEES is false, attach feet at these coordinates:
	// Pelvic structure Nodes 24-31 (Feet)
	addFootNodes(pelvicHex, footRingRadius, legOffset, footHeight, toeHeight);

	// If KNEES is true, attach the Lower Legs:
	// Pelvic structure Nodes 32-39 (Lower Legs/Shins)
	addLegNodes(pelvicHex, legOffset, leg_ringRadius, bottomLegStart, -twist);
	pelvicHex.addNode(legOffset, 0, bottomLegEnd);
	addLegNodes(pelvicHex, -legOffset, leg_ringRadius, bottomLegStart, twist);
	pelvicHex.addNode(-legOffset, 0, bottomLegEnd);
	// Attach Feet at new coordinates:
	// Pelvic structure Nodes 40-47 (Feet)
	addFootNodes(pelvicHex, footRingRadius, legOffset, foot_Height, toe_Height);
}

// Creates all Top and Bottom Segment Rods
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

// Creates all Pelvic, Leg, and Foot Rods
void addLegPairs(tgStructure& structure)
{
	// Create the Pelvic Rods
	addHexPairs(structure, "pelvic");
	for (int i = 7; i < 13; i++)
	{
		structure.addPair(i, 13, "pelvic rod");
	}
	structure.addPair(7, 8, "pelvic ring rod");
	structure.addPair(9, 10, "pelvic ring rod");
	structure.addPair(7, 11, "pelvic ring rod");
	structure.addPair(8, 12, "pelvic ring rod");
	structure.addPair(9, 11, "pelvic ring rod");
	structure.addPair(10, 12, "pelvic ring rod");

	// Create the first Leg Segment
	for (int i = 16; i <= 18; i++)
	{
		structure.addPair(14, i, "top leg to ring rod");
		structure.addPair(15, i, "bottom leg to ring rod");
	}
	structure.addPair(16, 17, "leg ring rod");
	structure.addPair(17, 18, "leg ring rod");
	structure.addPair(18, 16, "leg ring rod");

	for (int i = 21; i <= 23; i++)
	{
		structure.addPair(19, i, "top leg to ring rod");
		structure.addPair(20, i, "bottom leg to ring rod");
	}
	structure.addPair(21, 22, "leg ring rod");
	structure.addPair(22, 23, "leg ring rod");
	structure.addPair(23, 21, "leg ring rod");

	// If KNEES is false:
	if (!KNEES)
	{	// Create Feet at the bottom of the first Leg Segment
		for (int i = 25; i <= 27; i++)
		{
			structure.addPair(24, i, "foot rod");
		}
		structure.addPair(25, 26, "foot ring rod");
		structure.addPair(26, 27, "foot ring rod");
		structure.addPair(27, 25, "foot ring rod");
		for (int i = 29; i <= 31; i++)
		{
			structure.addPair(28, i, "foot rod");
		}
		structure.addPair(29, 30, "foot rod");
		structure.addPair(30, 31, "foot rod");
		structure.addPair(31, 29, "foot rod");
	}
	// If KNEES is true:
	else
	{	// Create the second Leg Segment
		for (int i = 32; i <= 34; i++)
		{
			structure.addPair(35, i, "bottom leg rod");
		}
		structure.addPair(32, 33, "bottom leg ring rod");
		structure.addPair(33, 34, "bottom leg ring rod");
		structure.addPair(34, 32, "bottom leg ring rod");

		for (int i = 36; i <= 38; i++)
		{
			structure.addPair(39, i, "bottom leg rod");
		}
		structure.addPair(36, 37, "bottom leg ring rod");
		structure.addPair(37, 38, "bottom leg ring rod");
		structure.addPair(38, 36, "bottom leg ring rod");

		// Create Feet at the bottom of the second Leg Segment
		for (int i = 41; i <= 43; i++)
		{
			structure.addPair(40, i, "foot rod");
		}
		structure.addPair(41, 42, "foot ring rod");
		structure.addPair(42, 43, "foot ring rod");
		structure.addPair(43, 41, "foot ring rod");
		for (int i = 45; i <= 47; i++)
		{
			structure.addPair(44, i, "foot rod");
		}
		structure.addPair(45, 46, "foot ring rod");
		structure.addPair(46, 47, "foot ring rod");
		structure.addPair(47, 45, "foot ring rod");
	}

}

// Creates all the Rods
void addAllPairs(tgStructure& hexPrime, tgStructure& topHex,
		tgStructure& bottomHex, tgStructure& pelvicHex)
{
	// Center structure
	for (int i = 1; i < 7; i++)
	{
		hexPrime.addPair(i, 0, "base/left structure rod");
	}
	for (int i = 1; i < 6; i++)
	{
		hexPrime.addPair(i, i + 1, "base/left ring rod");
	}
	hexPrime.addPair(1, 6, "base/left ring rod");
	for (int i = 1; i < 7; i++)
	{
		hexPrime.addPair(i, i + 6, "center structure rod");
	}
	for (int i = 7; i < 12; i++)
	{
		hexPrime.addPair(i, i + 1, "base/right ring rod");
	}
	hexPrime.addPair(7, 12, "base/right ring rod");
	for (int i = 7; i < 13; i++)
	{
		hexPrime.addPair(i, 13, "base/right structure rod");
	}

	// Top structure
	addHexPairs(topHex, "top");

	// Bottom structure
	addHexPairs(bottomHex, "bottom");

	// Pelvic, Leg, and Foot Structures
	addLegPairs(pelvicHex);
}

// Builds a single Segment
void addSegment(tgStructure& spine, const tgStructure& structure,
		btVector3 offset, int i)
{
	tgStructure* const t = new tgStructure(structure);
	t->addTags(tgString("segment", i));
	t->move(offset);
	spine.addChild(t);
}

// Builds all the Segments
void addAllSegments(tgStructure& spine, const tgStructure& hexPrime,
		const tgStructure& topHex, const tgStructure& bottomHex,
		const tgStructure& pelvicHex, const int topSegments,
		const int bottomSegments)
{
	const int totalSegments = topSegments + bottomSegments;

	// Creates the Base Start Segment
	tgStructure* const t = new tgStructure(hexPrime);
	t->addTags(tgString("segment", 0));
	spine.addChild(t);

	// Creates all remaining Segments
	for (int i = 1; i <= totalSegments; ++i)
	{
		if (!PELVIS)
		{ 	// If PELVIS is false, create a normal Spine
			if (i <= topSegments)
			{	// Top Segments
				addSegment(spine, topHex, i * topOffset, i);
			}
			else
			{	// Bottom Segments
				addSegment(spine, bottomHex, (i - topSegments) * bottomOffset,
						i);
			}
		}
		else
		{	// If PELVIS is true add Pelvic, Leg, and
			// Foot structures to the bottom of the Spine
			if (i <= topSegments)
			{	// Top Segments
				addSegment(spine, topHex, i * topOffset, i);
			}
			else if (i < totalSegments)
			{	// Bottom Segments
				addSegment(spine, bottomHex, (i - topSegments) * bottomOffset,
						i);
			}
			else if (i == totalSegments)
			{	// Pelvic structure
				addSegment(spine, pelvicHex, (i - topSegments) * bottomOffset,
						i);
			}
		}
	}
}

//Attaches all Base Actuators
void addCenterActuators(tgStructure& spine, std::vector<tgStructure*> children,
		std::string j, int i)
{
	int node;
	// Retrieves the Nodes
	tgNodes prime = children[0]->getNodes();
	tgNodes S1 = children[i + 1]->getNodes();

	if (j == "top")
	{	// Attaches a Top Segment onto the Base structure
		node = 0;
		// Actuators with normal stiffness
		spine.addPair(prime[1], S1[6], "center/" + j + " actuator");
		spine.addPair(prime[1], S1[2], "center/" + j + " actuator");
		spine.addPair(prime[3], S1[2], "center/" + j + " actuator");
		spine.addPair(prime[3], S1[4], "center/" + j + " actuator");
		spine.addPair(prime[5], S1[4], "center/" + j + " actuator");
		spine.addPair(prime[5], S1[6], "center/" + j + " actuator");
	}
	else
	{	//Attaches a Bottom Segment onto the Base structure
		node = 13;
		// Actuators with normal stiffness
		spine.addPair(prime[8], S1[1], "center/" + j + " actuator");
		spine.addPair(prime[8], S1[3], "center/" + j + " actuator");
		spine.addPair(prime[10], S1[3], "center/" + j + " actuator");
		spine.addPair(prime[10], S1[5], "center/" + j + " actuator");
		spine.addPair(prime[12], S1[5], "center/" + j + " actuator");
		spine.addPair(prime[12], S1[1], "center/" + j + " actuator");
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
		spine.addPair(S1[0], S2[k], j + " side baseActuator");
	}
	// Accounts for the Actuator offset
	// between the Top and Bottom Segments
	if (j == "bottom")
	{
		S1 = children[i + 1]->getNodes();
		S2 = children[i]->getNodes();
	}

	// Actuators with normal stiffness
	spine.addPair(S1[1], S2[6], j + " side actuator");
	spine.addPair(S1[1], S2[2], j + " side actuator");
	spine.addPair(S1[3], S2[2], j + " side actuator");
	spine.addPair(S1[3], S2[4], j + " side actuator");
	spine.addPair(S1[5], S2[4], j + " side actuator");
	spine.addPair(S1[5], S2[6], j + " side actuator");

}

// Attaches all Leg and Foot Actuators
void addLegActuators(tgStructure& spine, std::vector<tgStructure*> children,
		int i)
{
	tgNodes leg = children[i]->getNodes();

	// Attaches Actuators from the Pelvis to the Hexagonal structure
	spine.addPair(leg[7], leg[1], "hex/pelvis actuator");
	spine.addPair(leg[7], leg[2], "hex/pelvis actuator");
	spine.addPair(leg[8], leg[3], "hex/pelvis actuator");
	spine.addPair(leg[8], leg[4], "hex/pelvis actuator");
	spine.addPair(leg[10], leg[4], "hex/pelvis actuator");
	spine.addPair(leg[10], leg[5], "hex/pelvis actuator");
	spine.addPair(leg[9], leg[6], "hex/pelvis actuator");
	spine.addPair(leg[9], leg[1], "hex/pelvis actuator");
	spine.addPair(leg[11], leg[1], "hex/pelvis actuator");
	spine.addPair(leg[12], leg[4], "hex/pelvis actuator");

	spine.addPair(leg[7], leg[0], "hex/pelvis baseActuator");
	spine.addPair(leg[9], leg[0], "hex/pelvis baseActuator");
	spine.addPair(leg[8], leg[0], "hex/pelvis baseActuator");
	spine.addPair(leg[10], leg[0], "hex/pelvis baseActuator");
	spine.addPair(leg[11], leg[0], "hex/pelvis baseActuator");
	spine.addPair(leg[12], leg[0], "hex/pelvis baseActuator");

	// Attaches Actuators from the top of the Legs to the Pelvis
	spine.addPair(leg[14], leg[7], "pelvis/leg baseActuator");
	spine.addPair(leg[14], leg[8], "pelvis/leg baseActuator");
	spine.addPair(leg[14], leg[0], "pelvis/leg baseActuator");
	spine.addPair(leg[19], leg[9], "pelvis/leg baseActuator");
	spine.addPair(leg[19], leg[10], "pelvis/leg baseActuator");
	spine.addPair(leg[19], leg[0], "pelvis/leg baseActuator");

	// Attaches Actuators from the Leg Rings to the Pelvis
	spine.addPair(leg[16], leg[8], "pelvis/ring actuator");
	spine.addPair(leg[16], leg[12], "pelvis/ring actuator");
	spine.addPair(leg[17], leg[12], "pelvis/ring actuator");
	spine.addPair(leg[17], leg[13], "pelvis/ring actuator");
	spine.addPair(leg[17], leg[11], "pelvis/ring actuator");
	spine.addPair(leg[18], leg[7], "pelvis/ring actuator");
	spine.addPair(leg[18], leg[11], "pelvis/ring actuator");

	spine.addPair(leg[21], leg[10], "pelvis/ring actuator");
	spine.addPair(leg[21], leg[12], "pelvis/ring actuator");
	spine.addPair(leg[22], leg[9], "pelvis/ring actuator");
	spine.addPair(leg[22], leg[11], "pelvis/ring actuator");
	spine.addPair(leg[23], leg[11], "pelvis/ring actuator");
	spine.addPair(leg[23], leg[13], "pelvis/ring actuator");
	spine.addPair(leg[23], leg[12], "pelvis/ring actuator");

	// If KNEES is false:
	if (!KNEES)
	{	// Attaches Actuators from the Leg Rings to
		// their corresponding Feet
		spine.addPair(leg[24], leg[15], "leg/foot actuator");
		spine.addPair(leg[25], leg[18], "ring/foot actuator");
		spine.addPair(leg[25], leg[17], "ring/foot actuator");
		spine.addPair(leg[26], leg[17], "ring/foot actuator");
		spine.addPair(leg[26], leg[16], "ring/foot actuator");
		spine.addPair(leg[27], leg[16], "ring/foot actuator");
		spine.addPair(leg[27], leg[18], "ring/foot actuator");

		spine.addPair(leg[28], leg[20], "leg/foot actuator");
		spine.addPair(leg[29], leg[23], "ring/foot actuator");
		spine.addPair(leg[29], leg[22], "ring/foot actuator");
		spine.addPair(leg[30], leg[22], "ring/foot actuator");
		spine.addPair(leg[30], leg[21], "ring/foot actuator");
		spine.addPair(leg[31], leg[21], "ring/foot actuator");
		spine.addPair(leg[31], leg[23], "ring/foot actuator");
	}
	// If KNEES is true:
	else
	{	// Attaches Actuators from the top Leg Rings
		// to their corresponding bottom Leg Rings
		spine.addPair(leg[15], leg[32], "leg/leg actuator");
		spine.addPair(leg[15], leg[33], "leg/leg actuator");
		spine.addPair(leg[15], leg[34], "leg/leg actuator");
		spine.addPair(leg[16], leg[34], "leg/leg actuator");
		spine.addPair(leg[16], leg[32], "leg/leg actuator");
		spine.addPair(leg[17], leg[32], "leg/leg actuator");
		spine.addPair(leg[17], leg[33], "leg/leg actuator");
		spine.addPair(leg[18], leg[33], "leg/leg actuator");
		spine.addPair(leg[18], leg[34], "leg/leg actuator");

		spine.addPair(leg[20], leg[36], "leg/leg actuator");
		spine.addPair(leg[20], leg[37], "leg/leg actuator");
		spine.addPair(leg[20], leg[38], "leg/leg actuator");
		spine.addPair(leg[21], leg[36], "leg/leg actuator");
		spine.addPair(leg[21], leg[37], "leg/leg actuator");
		spine.addPair(leg[22], leg[37], "leg/leg actuator");
		spine.addPair(leg[22], leg[38], "leg/leg actuator");
		spine.addPair(leg[23], leg[38], "leg/leg actuator");
		spine.addPair(leg[23], leg[36], "leg/leg actuator");

		// Attaches Actuators from the bottom Leg Segments
		// to their corresponding Feet
		spine.addPair(leg[35], leg[40], "leg/foot actuator");
		spine.addPair(leg[32], leg[43], "ring/foot actuator");
		spine.addPair(leg[32], leg[42], "ring/foot actuator");
		spine.addPair(leg[33], leg[42], "ring/foot actuator");
		spine.addPair(leg[33], leg[41], "ring/foot actuator");
		spine.addPair(leg[34], leg[41], "ring/foot actuator");
		spine.addPair(leg[34], leg[43], "ring/foot actuator");

		spine.addPair(leg[41], leg[35], "leg/toe actuator");		//
		spine.addPair(leg[42], leg[35], "leg/toe actuator");		//
		spine.addPair(leg[43], leg[35], "leg/toe actuator");		//

		spine.addPair(leg[39], leg[44], "leg/foot actuator");
		spine.addPair(leg[36], leg[47], "ring/foot actuator");
		spine.addPair(leg[36], leg[46], "ring/foot actuator");
		spine.addPair(leg[37], leg[46], "ring/foot actuator");
		spine.addPair(leg[37], leg[45], "ring/foot actuator");
		spine.addPair(leg[38], leg[45], "ring/foot actuator");
		spine.addPair(leg[38], leg[47], "ring/foot actuator");

		spine.addPair(leg[45], leg[39], "leg/toe actuator");		//
		spine.addPair(leg[46], leg[39], "leg/toe actuator");		//
		spine.addPair(leg[47], leg[39], "leg/toe actuator");		//
	}
}

// Adds all the actuators
void addAllActuators(tgStructure& spine, int topSegments, int bottomSegments)
{
	// Creates a vector containing all the Segments
	const std::vector<tgStructure*> children = spine.getChildren();
	int totalSegments = children.size();

	// Actuators linking the Base structure to the first Segment (child):
	// If there are Top Segments, attach the first Segment onto the top
	if (topSegments >= 1)
	{
		addCenterActuators(spine, children, "top", 0);
	}
	// If there are no Top Segments and there are Bottom Segments,
	// attach the first Segment onto the bottom
	else if (bottomSegments >= 1)
	{
		addCenterActuators(spine, children, "bottom", 0);
	}

	for (int i = 1; i <= totalSegments; i++)
	{	// Attach all remaining Top Segments
		if (i < topSegments)
		{
			addSegmentActuators(spine, children, "top", i);
		}
		// If the last Top Segment has been reached
		// and there are Bottom Segments:
		else if (i == topSegments && bottomSegments >= 1)
		{
			addCenterActuators(spine, children, "bottom", i);
		}
		else if (i < totalSegments - 1)
		{	// Attach all remaining Bottom Segments
			addSegmentActuators(spine, children, "bottom", i);
		}
		// If the last Bottom Segment has been reached
		// and PELVIS is true:
		if (i == totalSegments - 1 && PELVIS)
		{
			addLegActuators(spine, children, i);
		}
	}
}

/**
 Methods that are not currently in use:
 void mapActuators(StandingHexagonalSpineTestModel::ActuatorMap& actuatorMap,
 tgModel& model)
 {
 // Note that tags don't need to match exactly, we could create
 // supersets if we wanted to
 actuatorMap["inner left"]  = model.find<tgBasicActuator>("inner left actuator");
 actuatorMap["inner right"] = model.find<tgBasicActuator>("inner right actuator");
 actuatorMap["inner top"]   = model.find<tgBasicActuator>("inner top actuator");
 actuatorMap["outer left"]  = model.find<tgBasicActuator>("outer left actuator");
 actuatorMap["outer right"] = model.find<tgBasicActuator>("outer right actuator");
 actuatorMap["outer top"]   = model.find<tgBasicActuator>("outer top actuator");
 }

 void trace(const tgStructureInfo& structureInfo, tgModel& model)
 {
 std::cout << "StructureInfo:" << std::endl
 << structureInfo    << std::endl
 << "Model: "        << std::endl
 << model            << std::endl;
 }
 */

}	// namespace

void StandingHexagonalSpineTestModel::setup(tgWorld& world)
{
	// Create the Structures
	tgStructure spine, hexPrime, topHex, bottomHex, pelvicHex;
	// Add all Nodes and Rods
	addAllNodes(hexPrime, topHex, bottomHex, pelvicHex);
	addAllPairs(hexPrime, topHex, bottomHex, pelvicHex);

	// Move the Structures to their starting points
	hexPrime.move(startPoint);
	topHex.move(topHexStart);
	bottomHex.move(bottomHexStart);
	pelvicHex.move(bottomHexStart);

	// Create our Spine Segments and Attach all Actuators
	addAllSegments(spine, hexPrime, topHex, bottomHex, pelvicHex, m_topSegments,
			m_bottomSegments);
	addAllActuators(spine, m_topSegments, m_bottomSegments);

	// Create the build spec that uses tags to turn the structure into a real model
	tgBuildSpec spec;

	// Rod Configuration
	const tgRod::Config rodConfig(rodRadius, density);
	// Actuator Configurations
	tgBasicActuator::Config muscleConfig(stiffness, damping, pretension);
	tgBasicActuator::Config baseActuatorConfig(baseStiffness, damping,
			pretension);

	// Add all Configurations
	spec.addBuilder("rod", new tgRodInfo(rodConfig));
	spec.addBuilder("actuator", new tgBasicActuatorInfo(muscleConfig));
	spec.addBuilder("baseActuator",
			new tgBasicActuatorInfo(baseActuatorConfig));

	// Rotate the Spine
	const btVector3 point(0, getStartHeight(), 0);
	const btVector3 axis(1, 0, 0);
	spine.addRotation(point, axis, PI / 2.0);

	// Build and place structure into the World
	tgStructureInfo structureInfo(spine, spec);
	structureInfo.buildInto(*this, world);

	// Set up the World
	tgModel::setup(world);

//	// We could now use tgCast::filter or similar to pull out the models (e.g. actuators)
//	// that we want to control.
//	allActuators = tgCast::filter<tgModel, tgBasicActuator>(getDescendants());
//	mapActuators(actuatorMap, *this);
//	trace(structureInfo, *this);
}

// Retrieves the Mass of the entire structure
double StandingHexagonalSpineTestModel::getMass()
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

// Retrieves and sets the Start Height of the Model
double StandingHexagonalSpineTestModel::getStartHeight()
{
	// Establishes the initial Start Height as the height of the Base
	double startHeight = 4.0;
	// Foot height when KNEES or PELVIS is true
	double foot = 0.5;
	// Number of bottom segments
	double bottomSeg = m_bottomSegments;
	// Used to adjust the start height as more segments are added
	double x = 1.5;

//	// Variables used to calculate structure height
//	// Number of top segments
//	double topSeg = m_topSegments;
//	// Establishes the initial Structure Height as the height of the Base
//	double structureHeight = 4.0;

	// If KNEES is true:
	if (KNEES)
	{
		// Add Foot and double the Leg Segment height:
		startHeight += (legSegmentHeight * 2.0) + foot;
//		structureHeight += (legSegmentHeight * 2.0) + foot;
	}
	// If KNEES is false and PELVIS is true:
	else if (PELVIS)
	{
		// Add Foot and Leg Segment height:
		startHeight += legSegmentHeight + foot;
//		structureHeight += legSegmentHeight + foot;
	}

	// The top segments are added to the structure negatively, so we do not
	// need to account for them when calculating the start height
	// (the tip of the first top segment is at 0.0)

	// Add the (adjusted) heights of all Bottom Segments:
	for (int j = 0; j <= bottomSeg; j++)
	{
		startHeight += x;
		x -= 0.05;
	}

//	// Add the actual heights of all the segments:	(EDIT THIS)
//	if(bottomSeg >= 1)
//		structureHeight += 0.4;
//	if(topSeg >= 1)
//		structureHeight += 0.4;
//	structureHeight += (bottomSeg-1)*1.0;
//	structureHeight += (topSeg-1)*1.0;
//	std::cout << "The structure is " << structureHeight << " meters tall"
//			<< std::endl;

	return startHeight;
}

// Notifies controllers of the step
void StandingHexagonalSpineTestModel::step(double dt)
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
StandingHexagonalSpineTestModel::getActuators(const std::string& key) const
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

