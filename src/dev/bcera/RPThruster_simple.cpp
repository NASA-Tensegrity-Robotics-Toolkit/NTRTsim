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
 * @file T6Thruster.cpp
 * @brief Implementation of thruster actuation of T6Model.
 * @author Kyunam Kim
 * @version 1.0.0
 * $Id$
 */

// This module
#include "RPThruster.h"
// This application
#include "RPModel.h"
// This library
#include "core/tgCast.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
#include "core/tgBaseRigid.h"
#include "core/abstractMarker.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <cstdlib>
#include <cmath>
#include <limits>

namespace
{
  double sf = 10; // scaling factor
  double worldTime = 0.0; // clock for world time
  double shootTime = 0.0; // used for thrust timing in multiple hop scenarios
  double initiateThrustTime = 12; // wait time until thrust initiation
  double thrustPeriod; // duration of thrust on	
  double nominalThrust; // stores nominal thrust value of this simulation, i.e., initial value of oldThrust
  double nominalPhi; // stores nominal angle phi value of this simulation, i.e., initial value of thrust angle phi
  int numberOfHops = 0; // count the number of hops the robot made in this simulation	
  int launchFlag = 0;

  //##### BCera - moved from onStep to namespace, to initialize jetDirections onSetup
  double thrust = 0.0;
  //double phi = 45.0*M_PI/180.0;
  double phi =  90.0*M_PI/180.0;;
  double theta = 0.0;
}

RPThruster::RPThruster(const int thrust) :
  m_thrust(thrust) //##### BCera - What is happening here?
{
  if (thrust < 0)
    {
      throw std::invalid_argument("Negative thrust not allowed!");
    }
}

void RPThruster::onSetup(RPModel& subject)
{	
  std::cout << "------------------ On Setup: Simple Thruster-------------------" << std::endl;

  srand((unsigned)time(NULL));

  //##### BCera - Added ability to have multiple jetstreams
  phi = getOldPhi();
  theta = getOldTheta();
  for(int p=0; p<jetnumber;p++)
    {
      //DIRECTIONS BASED ON BODY COORDINATES
      //jetDirections.push_back(btVector3(cos(phi)*cos(theta), sin(phi), cos(phi)*sin(theta)));
      jetDirections.push_back(btVector3(0,1,0));
      //jetDirections.push_back(btVector3(0,1,0);
      force.push_back(btVector3(0,0,0));
    }
					 
  std::cout << "------------------ On Step -------------------" << std::endl;
}


void RPThruster::onStep(RPModel& subject, double dt)
{	
  if (dt <= 0.0)
    {
      throw std::invalid_argument("dt is not positive");
    }
  else
    {	
      if ( worldTime > initiateThrustTime && launchFlag==0)
	{
	  // get thrust applied in previous time step
	  /*
	  std::cout << "ENTER THRUST MAGNITUDE IN NEWTONS: " << std::endl;
	  std::cin >> thrust;
	  std::cout << "ENTER THRUST PERIOD IN SECONDS: " << std::endl;
	  std::cin >> thrustPeriod;
	  */
	  thrust = 500;
	  thrustPeriod = 0.5;
	  std::cout << thrust << " N THRUST ACTIVATED FOR " << thrustPeriod << " SEC" << std::endl;
	  launchFlag = 1;
	}
      //std::cout << force.size();
      if(shootTime < thrustPeriod && launchFlag==1)
	{
	  btRigidBody* Body = subject.getAllBaseRigids()[6]->getPRigidBody(); // payload body
	  btTransform rotation;
	  btQuaternion orientation = Body->getOrientation();
	  rotation.setRotation(orientation);
	  mass = 10.0; // maximum mass of the robot suggested in ESI proposal

	  // simulate thrust noise
	  double deltaThrust = generateGaussianNoise(0.0, 0.02); // mean, variance
	  // add noise to previous thrust to generate current time step thrust
	  thrust += deltaThrust;
	  // save current time step thrust for use in next time step
	  saveOldThrust(thrust);
			
	  // get thrust orientation unit vector from previous time step
	  // these angles are in radians
	  phi = getOldPhi();
	  theta = getOldTheta();
	  // simulate angle noises
	  double deltaPhi = generateGaussianNoise(0.0, 0.02);
	  double deltaTheta = generateGaussianNoise(0.0, 0.02);
	  // add disturbances to angles to generate current time step orientation
	  phi += deltaPhi;
	  theta += deltaTheta;
	  // save current time step angles for use in next time step
	  saveOldPhi(phi);
	  saveOldTheta(theta);		
	  // compute thrust orientation unit vector
	  //######BRIAN - CHANGED ORI SYNTAX FROM ORI(X,Y,Z)
	  //btVector3 ori = btVector3(0,-20,0);	
	  // print out thrust and angle change
	  //~ std::cout << worldTime << "	" << getOldThrust() << "	" << getOldPhi() << "	" << getOldTheta() 
	  //~ << "	 " << Body->getCenterOfMassPosition() << std::endl;					
					
	  // thrust needs to be scaled up when actually applied in bullet!

	  //########BRIAN - ADDED RELATIVE ORIENTATION WRT BODY
	  //rotation = Body->getOrientation();
	  
	  for(int k=0; k<force.size(); k++)
	    {
	      force[k] = sf*thrust/jetnumber*jetDirections[k]; // assume target is towards +X direction
	      force[k] = rotation*force[k];
	    }
	  btVector3 pos(0.0, 0.0, 0.0);
	  for(int m=0; m<force.size(); m++){
	    Body->applyForce(force[m], pos);
	  } 
	}
      
      else{
	thrust = 0;
      }

      /*
      btVector3 unit = btVector3(1,0,0);
      unit = rotation*unit;
      double angle_between = btDot(force[0],unit)/force[0].norm()/unit.norm();
      //std::cout << worldTime - dt << "," << angle_between << ",\n"; // write thrust to file
      //std::cout << unit.getX() << "\n";
      */

		
      // time elapse
      worldTime += dt;	
      if(launchFlag == 1)
	shootTime += dt;	    	    	
    }
}

void RPThruster::saveOldThrust(double thrust)
{
  oldThrust = thrust;
}
 
void RPThruster::saveOldPhi(double phi)
{
  oldPhi = phi;
}

void RPThruster::saveOldTheta(double theta)
{
  oldTheta = theta;
}
 
double RPThruster::getOldThrust()
{
  return oldThrust;
}

double RPThruster::getOldPhi()
{
  return oldPhi;
}

double RPThruster::getOldTheta()
{
  return oldTheta;
}

double RPThruster::computeNextTargetTheta(btVector3 currentLocation, btVector3 targetLocation)
{
  double deltaX = targetLocation[0] - currentLocation[0];
  double deltaZ = targetLocation[2] - currentLocation[2];
  return atan2(deltaZ, deltaX);
}
  
double RPThruster::generateGaussianNoise(double mu, double sigma)
{
  /**
   * (From Wikipedia)
   * The standard Box-Muller transform generates 
   * values from the standard normal distribution 
   * (i.e. standard normal deviates) with mean 0 and standard deviation 1. 
   * The implementation below in standard C++ generates values 
   * from any normal distribution with mean \mu and variance \sigma^2. 
   * If Z is a standard normal deviate, then X = Z\sigma + \mu will 
   * have a normal distribution with mean \mu and standard deviation \sigma. 
   */ 
  const double epsilon = std::numeric_limits<double>::min();
  const double two_pi = 2*M_PI;//2.0*3.14159265358979323846;

  static double z0, z1;
  static bool generate;
  generate = !generate;

  if (!generate)
    return z1 * sigma + mu;

  double u1, u2;
  do
    {
      u1 = rand() * (1.0 / RAND_MAX);
      u2 = rand() * (1.0 / RAND_MAX);
    }
  while ( u1 <= epsilon );

  z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
  z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
  return z0 * sigma + mu;
}
