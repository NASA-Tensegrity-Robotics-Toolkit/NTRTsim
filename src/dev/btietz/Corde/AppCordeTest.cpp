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
 * @file AppCordeTest.cpp
 * @brief Contains the definition function main() for testing the Corde
 * string model
 * @author Brian Mirletz
 * $Id$
 */

// This application
#include "CordeModel.h"
// This library
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics Library
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
// The C++ Standard Library
#include <iostream>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{

#if (1) // X Pos
	btVector3 startPos(0.0, 0.0, 0.0);
	btVector3 endPos  (10.0, 0.0, 0.0);
	
	// Setup for neither bending nor rotation
	btQuaternion startRot( 0, sqrt(2)/2.0, 0, sqrt(2)/2.0);
	btQuaternion endRot = startRot;
#else
	btVector3 startPos(0.0, 0.0, 0.0);
	btVector3 endPos  (0.0, 0.0, 10.0);
	
	// Setup for neither bending nor rotation
	btQuaternion startRot( 0, 0, 0, 1);
	btQuaternion endRot = startRot;
#endif	
	// Values for Rope from Spillman's paper
	const std::size_t resolution = 10;
	const double radius = 0.01;
	const double density = 1300;
	const double youngMod = 0.5;
	const double shearMod = 0.5;
	const double stretchMod = 20.0;
	const double springConst = 100.0 * pow(10, 3);
	const double gammaT = 10.0 * pow(10, -6);
	const double gammaR = 1.0 * pow(10, -6);
	CordeModel::Config config(resolution, radius, density, youngMod, shearMod,
								stretchMod, springConst, gammaT, gammaR);
	
	CordeModel testString(startPos, endPos, startRot, endRot, config);
	
	double t = 0.0;
	double dt = 0.0001;
	for (int i = 0; i < 10000; i++)
	{
		testString.step(dt);
		t += dt;
	}
	#ifdef BT_USE_DOUBLE_PRECISION
		std::cout << "Double precision" << std::endl;
	#else
		std::cout << "Single Precision" << std::endl;
	#endif
	
    return 0;
}
