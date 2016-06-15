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
 * @file RPModel.cpp
 * @brief Contains the implementation of class RocketPower.
 * @author Brian Cera, based on code from Kyunam Kim
 * @version 1.0.0
 * $Id$
 */

// This module
//#include "RPModel.h"
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
#include <iostream>
#include <fstream>

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

  //const double sf = 20;//scaling factor with respect to meter scale. E.g., centimeter scale is achieved by setting sf = 100
  //const double length_scale = 0.25; //1 makes 4 m long rods
  // In meter scale, the robot is too small, while in centimeter scale, the robot rotates freely (free energy!)	
  // Also, don't forget to change gravity scale in AppThruster.cpp and T6Thruster.cpp!

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
  } 
    c =
      {
	2700/pow(sf,3),//0.688,    // density (kg / length^3)
	0.0254*sf,//0.31,     // radius (length)
	600,//1192.5*10,//613.0,   // stiffness (kg / sec^2) was 1500
	500,    // damping (kg / sec)
	4*sf*length_scale,     // rod_length (length)
	.02*sf,      // rod_space (length)
	0.99,      // friction (unitless)
	0.1,     // rollFriction (unitless)
	0.0,      // restitution (?)
	150*sf, //610,        // pretension -> set to 4 * 613, the previous value of the rest length controller
	0,			// History logging (boolean)
	300*sf,   // maxTens
	.02, //sf,    // targetVelocity

	// Use the below values for earlier versions of simulation.
	// 1.006,    
	// 0.31,     
	// 300000.0, 
	// 3000.0,   
	// 15.0,     
	// 7.5,      
      };

} // namespace

RPModel::RPModel() : tgModel() 
{
}

RPModel::~RPModel()
{
}

void RPModel::setup(tgWorld& world)
{

  allAbstractMarkers=tgCast::filter<tgModel, abstractMarker> (getDescendants());

}

void RPModel::step(double dt)
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
  for(int k=1;k<36;k++){
    std::cout << allActuators[k]->getTension() << " ";
  }
  std::cout << std::endl;
  
}


void RPModel::onVisit(tgModelVisitor& r)
{
  tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& RPModel::getAllActuators() const
{
  return allActuators;
}


const std::vector<tgRod*>& RPModel::getAllRods() const
{	
  return allRods;
}

const std::vector<tgBaseRigid*>& RPModel::getAllBaseRigids() const
{	
  return allBaseRigids;
}

const std::vector<abstractMarker*>& RPModel::getAllAbstractMarkers() const
{	
  return allAbstractMarkers;
}
    
void RPModel::teardown()
{
  notifyTeardown();
  tgModel::teardown();
}
