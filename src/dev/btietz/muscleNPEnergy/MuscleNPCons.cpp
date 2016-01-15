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
#include "core/tgSpringCableActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgRod.h"
#include "core/tgBox.h"
#include "core/tgBaseRigid.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgBasicContactCableInfo.h"

#include "core/tgBulletSpringCable.h"
#include "core/tgSpringCableAnchor.h"

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
	
	s.addNode(-2, 2, 0);
	s.addNode(0, 2, 0);
	s.addNode(0, 2, 0);
	s.addNode(0, 4, 0);
	s.addNode(0, 12, 0);
	s.addNode(0, 14, 0);
	
	
	// Free rod to which we will apply motion
	s.addNode(-1, 6, 5);
	s.addNode(1, 6, 5);

	//s.addPair(0, 1, "rod");
	s.addPair(2, 3, "box");
	s.addPair(4, 5, "box");
	
	s.addPair(6, 7, "rod");

	//s.addPair(1, 2, "muscle");
	s.addPair(3, 4, "muscle");
	s.move(btVector3(0, 0, 0));


	// Move the structure so it doesn't start in the ground
	s.move(btVector3(0, 0, 0));
	s.addRotation(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 1.0, 0.0), M_PI/4);
	tgSpringCableActuator::Config muscleConfig(1000, 0, 0.0, false, 600000000);
	
	// Create the build spec that uses tags to turn the structure into a real model
	tgBuildSpec spec;
	
	spec.addBuilder("rod2", new tgRodInfo(rodConfig2));
	spec.addBuilder("box", new tgBoxInfo(boxConfig));
	spec.addBuilder("rod", new tgRodInfo(rodConfig));
	spec.addBuilder("muscle", new tgBasicContactCableInfo(muscleConfig));

	// Create your structureInfo
	tgStructureInfo structureInfo(s, spec);
	// Use the structureInfo to build ourselves
	structureInfo.buildInto(*this, world);
	// We could now use tgCast::filter or similar to pull out the
	// models (e.g. muscles) that we want to control.
	allRods.clear();
	allMuscles.clear();
	allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
	allRods = tgCast::filter<tgModel, tgBaseRigid> (getDescendants());
	
	btRigidBody* body = allRods[0]->getPRigidBody();
	btRigidBody* body2 = allRods[1]->getPRigidBody();
	
	// Apply initial impulse
	btVector3 impulse(0.5, 0.0, 0.5);
	body->applyCentralImpulse(impulse);
	body2->applyCentralImpulse(impulse);
	
	btRigidBody* body3 = allRods[2]->getPRigidBody();
	
	std::cout << body3->getCenterOfMassTransform () << std::endl;
	
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
	
	tgModel::step(dt);
	
	btVector3 vCom(0, 0, 0);
	btScalar mass = 0;
	btScalar energy = 0;
	for (std::size_t i = 0; i < allRods.size(); i++)
	{
		tgBaseRigid& ri = *(allRods[i]);
		btRigidBody* body = ri.getPRigidBody();
		btVector3 localVel = body->getLinearVelocity();
		vCom += localVel * ri.mass();
		energy += localVel.length2() * ri.mass();// + body->getAngularVelocity().length2() ;
		mass += ri.mass();
	}
	
	btVector3 forceSum(0.0, 0.0, 0.0);
	
	const std::vector<const tgSpringCableAnchor*>& anchorList = allMuscles[0]->getSpringCable()->getAnchors();
	int n = anchorList.size();
	for (std::size_t i = 0; i < n; i++)
	{
		forceSum += anchorList[i]->getForce();
	}
	
#if (0)	
    std::cout << "Time " << totalTime << std::endl;
	std::cout << "Momentum " << vCom << std::endl;
	std::cout << "Energy " << energy << std::endl;
	std::cout << "Other Momentum " << getMomentum() << std::endl;
	std::cout << "Force sum " << forceSum << std::endl;
	std::cout << "Length " << allMuscles[0]->getCurrentLength();
	std::cout << " Dist " << (anchorList[0]->getWorldPosition() - anchorList[n-1]->getWorldPosition()).length() << std::endl;
	std::cout << "Anchors: " << n << std::endl;
	
	if (energy > 20){
		std::cout << "Here!" << std::endl;
	}
#endif
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

double MuscleNPCons::getEnergy() const
{
	double energy = 0;
	btScalar mass = 0;
	for (std::size_t i = 0; i < allRods.size(); i++)
	{
		tgBaseRigid* ri = allRods[i];
		btRigidBody* body = ri->getPRigidBody();
		btVector3 localVel = body->getLinearVelocity();
		energy += localVel.length2() * ri->mass();// + body->getAngularVelocity().length2() ;
		mass += ri->mass();
	}
	
	return energy;
}

btVector3 MuscleNPCons::getMomentum() const
{
	btVector3 vCom(0, 0, 0);
	btScalar mass = 0;

	for (std::size_t i = 0; i < allRods.size(); i++)
	{
		tgBaseRigid* ri = allRods[i];
		btRigidBody* body = ri->getPRigidBody();
		btVector3 localVel = body->getLinearVelocity();
		vCom += localVel * ri->mass();
		mass += ri->mass();
	}
	
	return vCom;
}

btVector3 MuscleNPCons::getVelocityOfBody(int body_num) const
{
	assert(body_num < allRods.size() && body_num >= 0);
	tgBaseRigid* ri = allRods[body_num];
	btRigidBody* body = ri->getPRigidBody();
	
	return body->getLinearVelocity();
}
