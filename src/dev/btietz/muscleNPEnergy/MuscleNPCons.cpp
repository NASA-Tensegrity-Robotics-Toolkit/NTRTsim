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

#include "MuscleNPCons.h"
#include "core/tgModelVisitor.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorld.h"
#include "core/tgLinearString.h"
#include "core/tgBaseString.h"
#include "core/tgRod.h"
#include "core/tgBox.h"
#include "core/tgBaseRigid.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgLinearStringInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "dev/muscleNP/tgMultiPointStringInfo.h"

#include "core/Muscle2P.cpp"
#include "core/muscleAnchor.cpp"

// The Bullet Physics Library
#include "BulletDynamics/Dynamics/btRigidBody.h"

MuscleNPCons::MuscleNPCons()
{
}

MuscleNPCons::~MuscleNPCons()
{
}

void MuscleNPCons::setup(tgWorld& world)
{


	const double rodDensity = 1; // Note: This needs to be high enough or things fly apart...
	const double rodRadius = 0.25;
	const tgRod::Config rodConfig(rodRadius, rodDensity);
	const tgRod::Config rodConfig2(rodRadius, 0.0);
	const tgBox::Config boxConfig(rodRadius, rodRadius, rodDensity);
	
	tgStructure s;
	
	s.addNode(-2, 2.1, 0);
	s.addNode(0, 2.1, 0);
	s.addNode(10, 2.1, 0);
	s.addNode(12, 2.1, 0);
	s.addNode(22, 2.1, 0);
	s.addNode(24, 2.1, 0);
	
	
	// Free rod to which we will apply motion
	s.addNode(15, 3, 5);
	s.addNode(15, 1, 5);

	//s.addPair(0, 1, "rod");
	s.addPair(2, 3, "rod");
	s.addPair(4, 5, "rod");
	
	s.addPair(6, 7, "rod");

	//s.addPair(1, 2, "muscle");
	s.addPair(3, 4, "muscle");
	s.move(btVector3(0, 0, 0));


	// Move the structure so it doesn't start in the ground
	s.move(btVector3(0, 0, 0));
	s.addRotation(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 1.0, 0.0), 0.0);
	tgBaseString::Config muscleConfig(1000, 0, false, 0, 600000000);
	
	// Create the build spec that uses tags to turn the structure into a real model
	tgBuildSpec spec;
	spec.addBuilder("rod", new tgRodInfo(rodConfig));
	spec.addBuilder("rod2", new tgRodInfo(rodConfig2));
	spec.addBuilder("box", new tgBoxInfo(boxConfig));
#if (1)
	spec.addBuilder("muscle", new tgMultiPointStringInfo(muscleConfig));
#else
	spec.addBuilder("muscle", new tgLinearStringInfo(muscleConfig));
#endif
	// Create your structureInfo
	tgStructureInfo structureInfo(s, spec);
	// Use the structureInfo to build ourselves
	structureInfo.buildInto(*this, world);
	// We could now use tgCast::filter or similar to pull out the
	// models (e.g. muscles) that we want to control.
	allMuscles = tgCast::filter<tgModel, tgLinearString> (getDescendants());
	allRods = tgCast::filter<tgModel, tgBaseRigid> (getDescendants());
	
	btRigidBody* body = allRods[2]->getPRigidBody();
	
	// Apply initial impulse
	btVector3 impulse(0.0, 0.0, -1.0);
	body->applyCentralImpulse(impulse);
	
	notifySetup();
	totalTime = 0.0;
	tgModel::setup(world);
}

void MuscleNPCons::teardown()
{
	tgModel::teardown();
}

void MuscleNPCons::step(double dt)
{
	totalTime += dt;
	
	btVector3 vCom(0, 0, 0);
	btScalar mass = 0;
	btScalar energy = 0;
	for (std::size_t i = 0; i < allRods.size(); i++)
	{
		tgBaseRigid& ri = *(allRods[i]);
		btRigidBody* body = ri.getPRigidBody();
		vCom += body->getVelocityInLocalPoint(btVector3(0.0, 0.0, 0.0)) * ri.mass();
		mass += ri.mass();
	}
	
	btVector3 forceSum(0.0, 0.0, 0.0);
	
	const std::vector<muscleAnchor*>& anchorList = allMuscles[0]->getMuscle()->getAnchors();
	for (std::size_t i = 0; i < anchorList.size(); i++)
	{
		forceSum += anchorList[i]->force;
	}
	
	
	std::cout << "Momentum " << vCom << std::endl;
	std::cout << "Force sum " << forceSum << std::endl;
	tgModel::step(dt);
}

/**
* Call tgModelVisitor::render() on self and all descendants.
* @param[in,out] r a reference to a tgModelVisitor
*/
void MuscleNPCons::onVisit(const tgModelVisitor& r) const
{
	r.render(*this);
	tgModel::onVisit(r);
}
