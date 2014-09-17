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

#include "simpleCordeTensegrity.h"

#include "core/tgModelVisitor.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorld.h"
#include "core/tgLinearString.h"
#include "core/tgBaseString.h"

#include "dev/Corde/CordeModel.h"
#include "dev/Corde/cordeCollisionObject.h"
#include "dev/Corde/cordeDynamicsWorld.h"

#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgLinearStringInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"

#include "dev/btietz/tgCordeModel.h"
#include "dev/btietz/tgCordeStringInfo.h"

simpleCordeTensegrity::simpleCordeTensegrity()
{
	
}
    
simpleCordeTensegrity::~simpleCordeTensegrity()
{
    
}
    
void simpleCordeTensegrity::setup(tgWorld& world)
{
#if (0)	
	// Values for Rope from Spillman's paper
	const std::size_t resolution = 20;
	const double radius = 0.01;
	const double density = 1300;
	const double youngMod = 0.5 * pow(10, 4);
	const double shearMod = 0.5 * pow(10, 4);
	const double stretchMod = 20.0 * pow(10, 6);
	const double springConst = 10.0 * pow(10, 1); 
	const double gammaT = 100.0 * pow(10, -4); // Position Damping
	const double gammaR = 1.0 * pow(10, -4); // Rotation Damping
#else
	#if (0)
		// Values for wire
			const std::size_t resolution = 20;
		const double radius = 0.001;
		const double density = 7860;
		const double youngMod = 200.0 * pow(10, 6);
		const double shearMod = 100.0 * pow(10, 6);
		const double stretchMod = 100.0 * pow(10, 6);
		const double springConst = 300.0 * pow(10, 3);
		const double gammaT = 0.05 * pow(10, -6); // Position Damping
		const double gammaR = 0.01 * pow(10, -6); // Rotation Damping
	#else
		// Values for thread
		const std::size_t resolution = 10;
		const double radius = 0.001;
		const double density = 1300;
		const double youngMod = 1 * pow(10, 1);
		const double shearMod = 1 * pow(10, 1);
		const double stretchMod = 60.0 * pow(10, 6);
		const double springConst = 0.1 * pow(10, 0); 
		const double gammaT = 5.0 * pow(10, -4); // Position Damping
		const double gammaR = 0.5 * pow(10, -4); // Rotation Damping
	#endif // Wire vs thread
#endif // Rope vs others
	CordeModel::Config cordeConfig(resolution, radius, density, youngMod, shearMod,
								stretchMod, springConst, gammaT, gammaR);
								
    const double rodDensity = 1;  // Note: This needs to be high enough or things fly apart...
    const double rodRadius  = 0.25;
    const tgRod::Config rodConfig(rodRadius, rodDensity);
	
	const tgRod::Config rodConfig2(rodRadius, 0.0);
	
	tgStructure s;
#if (0)
	s.addNode(0,0,0);
	s.addNode(0,3,0);
	s.addNode(2,5,0);
	s.addNode(-2,5,0);
	
	s.addNode(0, 10, 0);
	s.addNode(0, 7, 0);
	s.addNode(0, 5, 2);
	s.addNode(0, 5,-2);
	
	s.addPair(0, 1, "rod");
	s.addPair(1, 2, "rod");
	s.addPair(1, 3, "rod");

	s.addPair(4, 5, "rod");
	s.addPair(5, 6, "rod");
	s.addPair(5, 7, "rod");

	s.addPair(2, 6, "muscle");
	s.addPair(3, 7, "muscle");
	s.addPair(2, 7, "muscle");
	s.addPair(3, 6, "muscle");
	
	s.move(btVector3(0, 10, 0));
#else

	s.addNode(-5, 5, 0);
	s.addNode(0, 5, 0);
	s.addNode(10, 5, 0);
	s.addNode(15, 5, 0);

	s.addPair(0, 1, "rod2");
	s.addPair(2, 3, "rod2");
	
	s.addPair(1, 2, "muscle");
	
	s.move(btVector3(0, 0, 0));
#endif	
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 0, 0));
    
    
    tgBaseString::Config muscleConfig(1000, 0, false, 0);
    
    tgCordeModel::Config cordeModelConfig ( muscleConfig, cordeConfig);
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
       
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("rod2", new tgRodInfo(rodConfig2));
#if (0)
    spec.addBuilder("muscle", new tgLinearStringInfo(muscleConfig));
#else    
    spec.addBuilder("muscle", new tgCordeStringInfo(cordeModelConfig));
#endif    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

	// We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgCordeModel> (getDescendants());

    notifySetup();
    tgModel::setup(world);
}

void simpleCordeTensegrity::teardown()
{

    tgModel::teardown();
}
    
void simpleCordeTensegrity::step(double dt)
{
	allMuscles[0]->setRestLength(14.0, dt);
	tgModel::step(dt);
}
/**
* Call tgModelVisitor::render() on self and all descendants.
* @param[in,out] r a reference to a tgModelVisitor
*/
void simpleCordeTensegrity::onVisit(const tgModelVisitor& r) const
{
    r.render(*this);
    tgModel::onVisit(r);
}
