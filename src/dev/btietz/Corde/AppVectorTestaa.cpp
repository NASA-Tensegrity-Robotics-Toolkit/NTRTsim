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
 * @file AppVectorTest.cpp
 * @brief Testing the normals of vectors with simple i/o
 * @author Brian Mirletz
 * $Id$
 */

#include "tgcreator/tgUtil.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

#include <iostream>
#include <stdlib.h> 
 
using namespace std; 
 
int main(int argc, char** argv)
{
	std::string::size_type sz;
	
	btVector3 point1(atof(argv[1]), atof(argv[2]), atof(argv[3]));
	btVector3 point2(atof(argv[4]), atof(argv[5]), atof(argv[6]));

	cout << point1 << " " << point2 << endl;
	
	btVector3 zAxis(0.0, 0.0, 1.0);
	btVector3 axisVec = (point2 - point1).normalize();
	/*
	if (acos(zAxis.dot(axisVec)) > M_PI / 2.0)
	{
		axisVec = (m_massPoints[i]->pos - m_massPoints[i+1]->pos).normalize();
	}
	*/
	std::cout << zAxis.cross(axisVec).normalize() << std::endl;
	std::cout << acos(zAxis.dot(axisVec)) << std::endl;
	
	btQuaternion currentAngle( zAxis.cross(axisVec).normalize(), acos(zAxis.dot(axisVec)));
	
	btScalar q11 = currentAngle[0];
	btScalar q12 = currentAngle[1];
	btScalar q13 = currentAngle[2];
	btScalar q14 = currentAngle[3];
	
	
	const btVector3 director( (2.0 * (q11 * q13 + q12 * q14)),
				(2.0 * (q12 * q13 - q11 * q14)),
   ( -1.0 * q11 * q11 - q12 * q12 + q13 * q13 + q14 * q14));
	
	cout << "Quaternion " << currentAngle << endl;
	cout << "Director " << director << endl;
	cout << "Original " << axisVec << endl;
}
