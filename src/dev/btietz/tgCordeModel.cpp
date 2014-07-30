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

#include "dev/Corde/CordeModel.h"


tgCordeModel::tgCordeModel()
{

}
    
tgCordeModel::~tgCordeModel()
{
    
}
    
void tgCordeModel::setup(tgWorld& world)
{

	btVector3 startPos(0.0, 10.0, 0.0);
	btVector3 endPos  (10.0, 10.0, 0.0);
	
	// Setup for neither bending nor rotation
	btQuaternion startRot( 0, sqrt(2)/2.0, 0, sqrt(2)/2.0);
	btQuaternion endRot = startRot;

	// Values for Rope from Spillman's paper
	const std::size_t resolution = 10;
	const double radius = 0.01;
	const double density = 1300;
	const double youngMod = 0.5 * pow(10, 6);
	const double shearMod = 0.5 * pow(10, 6);
	const double stretchMod = 20.0 * pow(10, 6);
	const double springConst = 100.0 * pow(10, 3);
	const double gammaT = 10.0 * pow(10, -6);
	const double gammaR = 1.0 * pow(10, -6);
	CordeModel::Config config(resolution, radius, density, youngMod, shearMod,
								stretchMod, springConst, gammaT, gammaR);
	
	testString = new CordeModel(startPos, endPos, startRot, endRot, config);
}

void tgCordeModel::teardown()
{
    delete testString;
}
    
void tgCordeModel::step(double dt)
{
	//testString->applyForce(btVector3(-9.81, 0.0, 0.0), 0);
    testString->step(dt);
}
/**
* Call tgModelVisitor::render() on self and all descendants.
* @param[in,out] r a reference to a tgModelVisitor
*/
void tgCordeModel::onVisit(const tgModelVisitor& r) const
{
    r.render(*this);
}
