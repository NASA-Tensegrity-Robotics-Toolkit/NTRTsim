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

#include "soloCorde.h"

#include "core/tgModelVisitor.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorld.h"
#include "core/tgTags.h"

#include "dev/Corde/CordeModel.h"
#include "dev/Corde/cordeCollisionObject.h"
#include "dev/Corde/cordeDynamicsWorld.h"

#include "dev/btietz/tgCordeModel.h"
#include "dev/btietz/tgCordeStringInfo.h"

soloCorde::soloCorde() :
totalTime(0.0)
{
	
}
    
soloCorde::~soloCorde()
{
    
}
    
void soloCorde::setup(tgWorld& world)
{
#if (1)	
	// Values for Rope from Spillman's paper
	const std::size_t resolution = 40;
	const double radius = 0.01;
	const double density = 1300;
	const double youngMod = 0.5 * pow(10, 6);
	const double shearMod = 0.5 * pow(10, 6);
	const double stretchMod = 20.0 * pow(10, 6);
	const double springConst = 10.0 * pow(10, 3); 
	const double gammaT = 1.0 * pow(10, -6); // Position Damping
	const double gammaR = 1.0 * pow(10, -6); // Rotation Damping
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
		const std::size_t resolution = 40;
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

	btVector3 to(0, 5, 0);
	btVector3 from(10, 5, 0);

    tgBaseString::Config muscleConfig(1000, 0, false, 0, 600000000);
    
    tgCordeModel::Config cordeModelConfig ( muscleConfig, cordeConfig);
    
    std::vector<btVector3> startPositions = generatePoints(from, to, cordeConfig.resolution);
    
    cordeCollisionObject* p_cordeString = new cordeCollisionObject(startPositions, world, cordeConfig);
    
    cordeDynamicsWorld& dynamicsWorld =
                tgBulletUtil::worldToCordeDynamicsWorld(world);
	
	dynamicsWorld.addSoftBody(p_cordeString);
	
	tgTags cordeTag("myString");
	
	m_string = new tgCordeModel(p_cordeString, muscleConfig, cordeTag);
	
    notifySetup();
    tgModel::setup(world);
    
    totalTime = 0.0;
}

void soloCorde::teardown()
{

    tgModel::teardown();
}
    
void soloCorde::step(double dt)
{
	totalTime += dt;

	if (totalTime <= 0.5)
	{	
		//allMuscles[0]->setRestLength(5.0, dt);
		
		//allMuscles[1]->setRestLength(3.0, dt);
	#if (0)	
		m_string->getModel()->applyVecTorque(btVector3(0.0, 10.0, 0.0), 0);
		m_string->getModel()->applyVecTorque(btVector3(0.0, 10.0, 0.0), 38);
	#else

		m_string->getModel()->applyForce(btVector3(0.0, 0.0, 0.0), 0);
		m_string->getModel()->applyForce(btVector3(0.0, 10.0, 0.0), 39);	
	#endif
	}
#if (0)	
	std::cout << m_string->centerOfMass() << std::endl;
#else
	std::cout << totalTime << " " << m_string->energy() << std::endl;
#endif	
	tgModel::step(dt);
}
/**
* Call tgModelVisitor::render() on self and all descendants.
* @param[in,out] r a reference to a tgModelVisitor
*/
void soloCorde::onVisit(const tgModelVisitor& r) const
{
    r.render(*m_string);
    tgModel::onVisit(r);
}


std::vector<btVector3> soloCorde::generatePoints(btVector3& point1, 
													btVector3& point2,
													std::size_t resolution)
{
	std::vector<btVector3> points;
	
	points.push_back(point1);
	
    btVector3 rodLength(point2 - point1);
    btVector3 unitLength( rodLength / ((double) resolution - 1) );
    
    btVector3 massPos(point1);
    
    for (std::size_t i = 1; i < resolution; i++)
    {
		massPos += unitLength;
        points.push_back(massPos);
	}
    
    assert(points.size() == resolution);

    return points;
}
