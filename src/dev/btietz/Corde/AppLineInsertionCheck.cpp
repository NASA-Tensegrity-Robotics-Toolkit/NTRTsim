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
 * @file AppLineInsertionCheck.cpp
 * @brief Simple tests for determining the order of points in space
 * @author Brian Mirletz
 * $Id$
 */

#include "tgcreator/tgUtil.h"

// The Bullet Physics Library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
// The C++ Standard Library
#include <iostream>
#include <cmath>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
	btVector3 point1(atof(argv[1]), atof(argv[2]), atof(argv[3]));
	btVector3 point2(atof(argv[4]), atof(argv[5]), atof(argv[6]));

	btVector3 point3(atof(argv[7]), atof(argv[8]), atof(argv[9]));
#if (1)
	btVector3 normal(atof(argv[10]), atof(argv[11]), atof(argv[12]));
#else
	btVector3 point4(0.0, 0.0, 0.0);	
#endif
	std::cout << point1 << " " << point2 << " " << point3 << std::endl;

	std::cout << "new point " << (point3 - point1).dot(point2) << std::endl;
	std::cout << "old point " << (point3 - point1).dot(point3) << std::endl;
	
	btVector3 lineA = point3 - point2;
	btVector3 lineB = point1 - point2;
	
	normal.normalize();
	
#if (1)	
	btVector3 tangentDir =( (lineB - lineA).cross(normal)).normalize();
	btVector3 tangentMove = (lineB + lineA).dot(tangentDir) * tangentDir / 2.0;
	btVector3 newPos = point2 + tangentMove;
	
	std::cout << "tangentDir " << tangentDir << std::endl;
	std::cout << "tangentMove " << tangentMove << std::endl;
	std::cout << "newPos " << newPos << std::endl;
#else
	btQuaternion q1 = tgUtil::getQuaternionBetween(lineA, normal);
	btQuaternion q2 = tgUtil::getQuaternionBetween(normal, lineB);
	btQuaternion q3 = tgUtil::getQuaternionBetween(lineB, lineA);
	
	btVector3 lineACopy = lineA;
	btVector3 lineBCopy = lineB;
	btVector3 ab = lineA + lineB;
	btVector3 abNorm = (lineACopy.normalize() + lineBCopy.normalize()).normalize();
	
	// Project normal into AB plane
	btVector3 AN = lineA.dot(normal) * lineACopy.normalize();
	btVector3 BN = lineB.dot(normal) * lineBCopy.normalize();
	btVector3 normalProjection = abNorm.dot(normal) * abNorm;
	
	normalProjection.normalize();
	
	std::cout << "lineA Norm: " << lineACopy << std::endl;
	std::cout << "lineB Norm: " << lineBCopy << std::endl;
	std::cout << "Normal: " << normal << std::endl;
	std::cout << "Projection: " << normalProjection << std::endl;
	std::cout << "Angle AN: " << lineA.angle(normalProjection) << std::endl;
	std::cout << "Angle BN " << lineB.angle(normalProjection) << std::endl;
	std::cout << "Sum AN + BN " << lineA.angle(normalProjection) + lineB.angle(normalProjection) << std::endl;
	std::cout << "Angle AB " << lineA.angle(lineB) << std::endl;
	
	std::cout << "Angle normal w/ x axis " << btVector3(1.0, 0.0, 0.0).angle(normal) << std::endl;
	std::cout << "Angle lineA w/ x axis " << btVector3(1.0, 0.0, 0.0).angle(lineA) << std::endl;
	std::cout << "Angle lineB w/ x axis " << btVector3(1.0, 0.0, 0.0).angle(lineB) << std::endl;
	
	std::cout << "Traditional: " << lineA.dot(normal) << " " << lineB.dot(normal) << std::endl;
	
#endif	
    return 0;
}
