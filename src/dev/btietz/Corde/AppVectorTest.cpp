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

#include <iostream>
#include <stdlib.h> 
 
using namespace std; 
 
int main(int argc, char** argv)
{
	std::string::size_type sz;
	
	btVector3 point1(atof(argv[1]), atof(argv[2]), atof(argv[3]));
	btVector3 point2(atof(argv[4]), atof(argv[5]), atof(argv[6]));
	btVector3 point3(atof(argv[7]), atof(argv[8]), atof(argv[9]));
	
	cout << point1 << " " << point2 << " " << point3 << endl;
	
	btVector3 unit = (point2 - point1).normalize();
	
	btScalar a = unit[0];
	btScalar b = unit[1];
	btScalar c = unit[2];
	// Find an arbitrary perpendicular vector
	btVector3 perp1 = btVector3(b - c, -a, a).normalize(); 
	
	// Find one perpendicular to both
	btVector3 perp2 = perp1.cross(unit);
	
	cout << perp1 << " " << perp2 << endl;
	cout << perp1.dot(unit) << " " << perp2.dot(unit) << " " << perp2.dot(perp1) << endl;
	
	btVector3 unit2 = (point3 - point2).normalize();
	
	a = unit2[0];
	b = unit2[1];
	c = unit2[2];
	// Find an arbitrary perpendicular vector
	btVector3 perp3 = btVector3(b - c, -a, a).normalize(); 
	
	// Find one perpendicular to both
	btVector3 perp4 = perp3.cross(unit2);
	
	cout << perp3 << " " << perp4 << endl;
	cout << perp3.dot(unit2) << " " << perp4.dot(unit2) << " " << perp3.dot(perp4) << endl;
}
