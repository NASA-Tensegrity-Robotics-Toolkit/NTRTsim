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
	btVector3 point3(atof(argv[7]), atof(argv[8]), atof(argv[9]));
	btVector3 point4(atof(argv[10]), atof(argv[11]), atof(argv[12]));
	
	cout << point1 << " " << point2 << " " << point3 << endl;
	
	btVector3 unit = (point2 - point1).normalize();
	
	btVector3 unit2 = (point3 - point2).normalize();
	btVector3 unit3 = (point4 - point3).normalize();
	
	btVector3 perp1, perp2, perp3, perp4, perp5, perp6;
	btScalar a, b, c;
	
	if (unit.dot(unit2) > 1.f - FLT_EPSILON)
	{
		a = unit[0];
		b = unit[1];
		c = unit[2];
		// Find an arbitrary perpendicular vector
		if (a != 0 && b != c)
		{
			perp1 = btVector3(b - c, -a, a).normalize();
		}
		else
		{
			perp1 = btVector3(-b, a - c, b).normalize();
		}
	}
	else
	{ 
		perp1 = unit.cross(unit2).normalize();
	}
	
	// Find one perpendicular to both
	perp2 = perp1.cross(unit).normalize();
	
	a = unit2[0];
	b = unit2[1];
	c = unit2[2];
	// Find an arbitrary perpendicular vector
	perp3 = perp1; 
	
	// Find one perpendicular to both
	perp4 = perp3.cross(unit2).normalize();
	
	if (unit.dot(unit2) > 1.f - FLT_EPSILON)
	{
		a = unit[0];
		b = unit[1];
		c = unit[2];
		// Find an arbitrary perpendicular vector
		if (a != 0 && b != c)
		{
			perp5 = btVector3(b - c, -a, a).normalize();
		}
		else
		{
			perp5 = btVector3(-b, a - c, b).normalize();
		}
	}
	else
	{ 
		perp5 = unit2.cross(unit3).normalize();
	}
	
	
	btScalar x, y, z, w;
	// Compute quaternions - testing method in paper
	btScalar q4sqr = 0.25 * (1 + perp2[0] + perp1[1] + unit[2]);
	if (q4sqr > FLT_EPSILON)
	{
		w = sqrt(q4sqr);
		x = (unit[1] - perp1[2]) / (4.0 * w);
		y = (perp2[2] - unit[0]) / (4.0 * w);
		z = (perp1[0] - perp2[1]) / (4.0 * w);
	}
	else
	{
		w = 0;
		btScalar q1sqr = - 0.5 * (perp2[1] + unit[2]);
		if (q1sqr > FLT_EPSILON)
		{
			x = sqrt(q1sqr);
			y = perp2[1] / (2.0 * x);
			z = perp2[2] / (2.0 * x);
		}
		else
		{
			x = 0;
			btScalar q2sqr = 0.5 * (1 - unit[2]);
			if (q2sqr > FLT_EPSILON)
			{
				y = sqrt(q2sqr);
				z = perp1[2] / (2.0 * y);
			}
			else
			{
				y = 0;
				z = 1;
			}
		}
	}
	btQuaternion qtOut(x, y, z, w);
	
	perp6 = perp5.cross(unit3).normalize();
	
	cout << "Unit Vectors" << endl;
	cout << unit << " " << unit2 << " " << unit3 << endl;
	cout << unit.dot(unit2) << " " << unit2.dot(unit3) << endl;
	
	cout << perp1 << " " << perp2 << endl;
	cout << perp1.dot(unit) << " " << perp2.dot(unit) << " " << perp2.dot(perp1) << endl;
	
	cout << perp3 << " " << perp4 << endl;
	cout << perp3.dot(unit2) << " " << perp4.dot(unit2) << " " << perp3.dot(perp4) << endl;
	
	cout << perp5 << " " << perp6 << endl;
	cout << perp5.dot(unit3) << " " << perp6.dot(unit3) << " " << perp5.dot(perp6) << endl;
	
	cout << "Test lack of torsion" << endl;
	cout << perp1.dot(perp3) << " " << perp2.dot(perp4) << " " << perp1.dot(perp4) << " " << perp2.dot(perp3) << endl;
	cout << perp3.dot(perp5) << " " << perp4.dot(perp6) << " " << perp3.dot(perp6) << " " << perp4.dot(perp5) << endl;
	
	cout << qtOut << endl;
}
