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
 * @file T6Model.cpp
 * @brief Contains the implementation of class T6Model.
 * $Id$
 */

// This module
#include "T6Model.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
#include <cmath>

namespace
{
    // see tgBasicActuator and tgRod for a descripton of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    // Note: This current model of the SUPERball rod is 1.5m long by 3 cm radius,
    // which is 0.00424 m^3.
    // For SUPERball v1.5, mass = 3.5kg per strut, which comes out to 
    // 0.825 kg / (decimeter^3).

    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double damping;
        double rod_length;
        double rod_space;    
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        bool   hist;
        double maxTens;
        double targetVelocity;
    } c =
   {
     0.688,    // density (kg / length^3)
     0.31,     // radius (length)
     613.0,   // stiffness (kg / sec^2) was 1500
     200.0,    // damping (kg / sec)
     16.84,     // rod_length (length)
     7.5,      // rod_space (length)
     0.99,      // friction (unitless)
     0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)
     0*2452.0,        // pretension -> set to 4 * 613, the previous value of the rest length controller
     0,			// History logging (boolean)
     100000,   // maxTens
     10000,    // targetVelocity

     // Use the below values for earlier versions of simulation.
     // 1.006,    
     // 0.31,     
     // 300000.0, 
     // 3000.0,   
     // 15.0,     
     // 7.5,      
  };
} // namespace

std::vector<double> azemuth_altitude_to_xyz(double azemuth, double altitude, double length,
                                             double bx, double by, double bz);

T6Model::T6Model() : tgModel() 
{
}

T6Model::~T6Model()
{
}



void T6Model::addNodes(tgStructure& s)
{
    double alt = M_PI/6;
    double tl1 = c.rod_length / 2;
    double h = c.rod_length/1.5;
    double jut = -M_PI/8;
    
    // Bottom triangle rods
    s.addNode(-tl1/2, 0, 0);            // 0
    std::vector<double> p = azemuth_altitude_to_xyz(-M_PI/3 + jut, alt, c.rod_length, -tl1/2, 0, 0);
    s.addNode(p[0], p[1], p[2]);        // 1
    s.addNode(0, -tl1*sqrt(3)/2, 0);    // 2
    p = azemuth_altitude_to_xyz(M_PI/3 + jut, alt, c.rod_length, 0, -tl1*sqrt(3)/2, 0);
    s.addNode(p[0], p[1], p[2]);        // 3
    s.addNode(tl1/2, 0, 0);             // 4
    p = azemuth_altitude_to_xyz(M_PI + jut, alt, c.rod_length, tl1/2, 0, 0);
    s.addNode(p[0], p[1], p[2]);        // 5
    
    double yshift = -tl1*sqrt(3)/3;
    // Upper triangle rods
    s.addNode(-tl1/2, yshift, h);       // 6
    p = azemuth_altitude_to_xyz(0 + jut, -alt, c.rod_length, -tl1/2, yshift, h);
    s.addNode(p[0], p[1], p[2]);        // 7
    s.addNode(tl1/2, yshift, h);        // 8
    p = azemuth_altitude_to_xyz(2*M_PI/3 + jut, -alt, c.rod_length, tl1/2, yshift, h);
    s.addNode(p[0], p[1], p[2]);        // 9
    s.addNode(0, tl1*sqrt(3)/2 + yshift, h);  // 10
    p = azemuth_altitude_to_xyz(-2*M_PI/3 + jut, -alt, c.rod_length, 0, tl1*sqrt(3)/2 + yshift, h);
    s.addNode(p[0], p[1], p[2]);        // 11
}

void T6Model::addRods(tgStructure& s)
{
    s.addPair( 0,  1, "rod");
    s.addPair( 2,  3, "rod");
    s.addPair( 4,  5, "rod");
    s.addPair( 6,  7, "rod");
    s.addPair( 8,  9, "rod");
    s.addPair( 10, 11, "rod");
}

void T6Model::addActuators(tgStructure& s)
{
    // Bottom triangle 
    s.addPair(0, 2,  "muscle"); // 0
    s.addPair(2, 4,  "muscle");
    s.addPair(4, 0,  "muscle");
    
    // Bottom prism vertical strings
    s.addPair(0, 5,  "muscle"); // 3
    s.addPair(2, 1,  "muscle");
    s.addPair(4, 3,  "muscle");
    
    // Top triangle 
    s.addPair(6, 8,  "muscle"); // 6
    s.addPair(8, 10,  "muscle");
    s.addPair(10, 6,  "muscle");
    
    // Top prism vertical strings
    s.addPair(6, 11,  "muscle"); // 9
    s.addPair(8, 7,  "muscle");
    s.addPair(10, 9,  "muscle");
    
    // Linking of top and bottom prism ends
    s.addPair(1, 7,  "muscle"); // 12
    s.addPair(7, 3,  "muscle");
    s.addPair(3, 9,  "muscle");
    s.addPair(9, 5,  "muscle");
    s.addPair(5, 11,  "muscle");
    s.addPair(11, 1,  "muscle");
    
    // Top prism to bottom prisms, end to beginning
    s.addPair(1, 6,  "muscle"); // 18
    s.addPair(3, 8,  "muscle");
    s.addPair(5, 10,  "muscle");
    s.addPair(11, 0,  "muscle");
    s.addPair(7, 2,  "muscle");
    s.addPair(9, 4,  "muscle");
}


void T6Model::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);
    
    /// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate
    // Muscle configuration
    tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension, c.hist, 
					    c.maxTens, c.targetVelocity);
            
    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addActuators(s);
    

    // Rotate the model in the world so the default view shows normal x, y, z orientation.
    btVector3 rotationPoint = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis = btVector3(1, 0, 0); // x-axis rotation
    double rotationAngle = M_PI/2;
    s.addRotation(rotationPoint, rotationAxis, rotationAngle);
    rotationPoint = btVector3(0, 0, 0); // origin
    rotationAxis = btVector3(0, 0, 1); // z-axis rotation
    rotationAngle = M_PI;
    s.addRotation(rotationPoint, rotationAxis, rotationAngle);
    
    s.move(btVector3(0, 10, 0)); // Move CoM to a certain point.
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void T6Model::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

std::vector<double> azemuth_altitude_to_xyz(double azemuth, double altitude, double length,
                                             double bx, double by, double bz)
{
    std::vector<double> xyz;
    xyz.push_back(length*cos(altitude)*cos(azemuth) + bx); // x
    xyz.push_back(length*cos(altitude)*sin(azemuth) + by); // y
    xyz.push_back(length*sin(altitude) + bz);              // z
    return xyz;
}

void T6Model::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& T6Model::getAllActuators() const
{
    return allActuators;
}
    
void T6Model::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
