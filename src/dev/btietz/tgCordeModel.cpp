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

#include "tgCordeModel.h"

#include "core/tgModelVisitor.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorld.h"

#include "dev/Corde/CordeModel.h"
#include "dev/Corde/cordeCollisionObject.h"
#include "dev/Corde/cordeDynamicsWorld.h"

tgCordeModel::tgCordeModel()
{

}
    
tgCordeModel::~tgCordeModel()
{
    
}
    
void tgCordeModel::setup(tgWorld& world)
{

	btVector3 startPos(0.0, 5.0, 0.0);
	btVector3 endPos  (10.0, 5.0, 0.0);
	
#if (0)	// Unused reference implementations from previous constructors
	// Setup for neither bending nor rotation note that (0, 0, 0, -1) fails to produce no bending
	btQuaternion startRot( 0.0, sqrt(2)/2.0, sqrt(2)/2.0, 0.0); // Y axis
	btQuaternion startRot( 0.0, 0.0, 0.0, 1.0); // Z axis
	btQuaternion startRot( 0.0, sqrt(2)/2.0, 0.0, sqrt(2)/2.0);
#endif
	
#if (1)	
	// Values for Rope from Spillman's paper
	const std::size_t resolution = 100;
	const double radius = 0.1;
	const double density = 1300;
	const double youngMod = 0.5 * pow(10, 6);
	const double shearMod = 0.5 * pow(10, 6);
	const double stretchMod = 20.0 * pow(10, 6);
	const double springConst = 100.0 * pow(10, 1); // Can't quite accomplish paper level stiffness without things blowing up.
	const double gammaT = 100.0 * pow(10, -6); // Position Damping
	const double gammaR = 1.0 * pow(10, -6); // Rotation Damping
#else
	// Values for thread
		const std::size_t resolution = 20;
	const double radius = 0.001;
	const double density = 1300;
	const double youngMod = 1.0 * pow(10, 1);
	const double shearMod = 1.0 * pow(10, 1);
	const double stretchMod = 0.02 * pow(10, 6);
	const double springConst = 0.1 * pow(10, 0);
	const double gammaT = 1.0 * pow(10, -6); // Position Damping
	const double gammaR = 1.0 * pow(10, -6); // Rotation Damping
#endif
	CordeModel::Config config(resolution, radius, density, youngMod, shearMod,
								stretchMod, springConst, gammaT, gammaR);
	
    std::vector<btVector3> startPositions = generatePoints(startPos, endPos, resolution);
    
    cordeDynamicsWorld& dynamicsWorld =
                tgBulletUtil::worldToCordeDynamicsWorld(world);
    
	testString = new cordeCollisionObject(startPositions, world, config);
	
	dynamicsWorld.addSoftBody(testString);
}

void tgCordeModel::teardown()
{
    // World handles deleting collision object
    testString = NULL;
    tgModel::teardown();
}
    
void tgCordeModel::step(double dt)
{
	//testString->applyForce(btVector3(0.0, -9.0, 0.0), 0);
	//testString->applyForce(btVector3(0.0, 9.0, 0.0), 19);
	testString->applyUniformAcc(btVector3(0.0, -9.81, 0.0));
	//testString->applyVecTorque(btVector3(0.0, -100.0, 0.0), 0);
	//testString->applyVecTorque(btVector3(0.0, 10.0, 0.0), 18);
    //testString->step(dt);
}
/**
* Call tgModelVisitor::render() on self and all descendants.
* @param[in,out] r a reference to a tgModelVisitor
*/
void tgCordeModel::onVisit(const tgModelVisitor& r) const
{
    r.render(*this);
}

std::vector<btVector3> tgCordeModel::generatePoints(btVector3& point1, 
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
