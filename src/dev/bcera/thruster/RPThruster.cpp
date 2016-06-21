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
#include "PrismModel.h"
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
  double sf = 1; // scaling factor
  double worldTime = 0.0; // clock for world time
  double shootTime = 0.0; // used for thrust timing in multiple hop scenarios
  double initiateThrustTime = 2.5; // wait time until thrust initiation
  double thrustPeriod = 5; // duration of thrust on
  bool thrusted = false; // records if this cycle of thrust has happened or not for each hop
  double targetDistance = 10.0; // distance to the target (before scaling)
  const btVector3 targetLocation = btVector3(targetDistance*sf, 0.0, 0.0); // target is located 1000m away in +X direction	
  bool doneHopping = false; // end hopping when this is true
  int numberOfHops = 0; // count the number of hops the robot made in this simulation	
  std::ofstream simlog; // log file for thruster related variables
  bool doLog = false; // choose to log thruster related variables
  double timePassed = 0.0; // controls logging frequency

  //##### BCera - moved from onStep to namespace, to initialize jetDirections onSetup
  double thrust = 1.0;
}

RPThruster::RPThruster(const int thrust) :
  m_thrust(thrust) 
{
  if (thrust < 0)
    {
      throw std::invalid_argument("Negative thrust not allowed!");
    }
}

void RPThruster::onSetup(PrismModel& subject)
{	
  std::cout << "------------------ On Setup -------------------" << std::endl;
    
  if ( doLog ) { simlog.open("./log/rotation.csv"); }

  srand((unsigned)time(NULL));

  //Added ability to have multiple jetstreams~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  jetnumber = 7;
  for(int p=0; p<jetnumber;p++)
    {
      //DIRECTIONS BASED ON BODY COORDINATES
      jetDirections.push_back(btVector3(0,0,1)); //Directional Unit Vectors (SAME FOR NOW/DEBUGGING)
      force.push_back(btVector3(0,0,0)); //Instantiate same number of force vectors
    }		
				 
  std::cout << "------------------ On Step -------------------" << std::endl;
}


void RPThruster::onStep(PrismModel& subject, double dt)
{	
  if (dt <= 0.0)
    {
      throw std::invalid_argument("dt is not positive");
    }
  else
    {	
      btRigidBody* thrusterRigidBody = subject.ThrusterBodies[0]; // payload body
      //std::vector<tgRod *> thrusterParts = PrismModel::find<tgRod>("thruster");
      //Get thruster transform
      //tgRod* thrusterRod = thrusterParts[0]; //Outer
      //btRigidBody* thrusterRigidBody = thrusterRod->getPRigidBody();
      btTransform rotation;
      btQuaternion orientation = thrusterRigidBody->getOrientation();
      rotation.setRotation(orientation);

      if ( shootTime > initiateThrustTime && shootTime <= (initiateThrustTime + thrustPeriod) )
	{
	  for(int k=0; k<force.size(); k++)
	    {
	      force[k] = sf*thrust/jetnumber*jetDirections[k]; // assume target is towards +X direction
	      force[k] = rotation*force[k]; //rotate to match thruster body
	    }

	  if (!thrusted) 
	    {
	      numberOfHops += 1;
	      std::cout << "Start thrusting! (# of hops = " << numberOfHops << ")" << std::endl;
	    }
	  thrusted = true; // notify that this cycle of thrust has been done
	}
		
      else
	{
	  for(int k=0; k<force.size(); k++)
	    {
	      force[k] = 0*force[k];
	    }			
	}

      //Logging commented out for now
      /*
      btVector3 unit = btVector3(1,0,0);
      unit = rotation*unit;
      double angle_between = btDot(force[0],unit)/force[0].norm()/unit.norm();
      //std::cout << worldTime - dt << "," << angle_between << ",\n"; // write thrust to file
      //std::cout << unit.getX() << "\n";
      */
      
      btVector3 pos(0.0, 0.0, 0.0);
      for(int m=0; m<force.size(); m++){
	thrusterRigidBody->applyForce(force[m], pos);
      }

      /*
      // determine when we want to do another hop and reset shootTime
      if (!doneHopping && thrusted && shootTime > (initiateThrustTime + thrustPeriod) && Body->getLinearVelocity().length() < 1.0)
	{
	  // Did the robot get close enough to the target?
	  if ( (targetLocation - Body->getCenterOfMassPosition()).length() < hopDistance*sf/2 )
	    {
	      doneHopping = true; // end hopping
	      std::cout << "Hopping is done!" << std::endl;
	      std::cout << "Final distance to the target is " 
			<< (targetLocation - Body->getCenterOfMassPosition()).length()/sf
			<< " meters." << std::endl;
	    }
	  // If not, continue to hop
	  else
	    {
	      oldThrust = nominalThrust; // reset thrust to initial value
	      oldPhi = nominalPhi; // reset to initial phi
	      oldTheta = computeNextTargetTheta(Body->getCenterOfMassPosition(), targetLocation); // reset heading orientation
	      std::cout << "Next heading direction : " << oldTheta*180.0/M_PI << std::endl;
	      std::cout << "Distance to target : " << (targetLocation - Body->getCenterOfMassPosition()).length()/sf << std::endl;				
	      for(int k=0; k<force.size(); k++)
		{
		  force[k] = 0*force[k];
		}	
	      
	      shootTime = 0.0; // reset clock for shootTime
	      thrusted = false; // reset thrusted boolean
	    }
	}
      */
		
      if (thrusterRigidBody->getCenterOfMassPosition()[1] < -10*sf)
	{
	  std::cout << "Robot fell outside of the world!" << std::endl;
	}
		
      // time elapse
      timePassed += dt;
      worldTime += dt;	
      shootTime += dt;
      /*
      if ( timePassed >= 0.01 && doLog )
    	{	
	  timePassed = dt;
	  //double angle = btDot(btVector3(cos(phi)*cos(theta), sin(phi), cos(phi)*sin(theta)),
	  //  btVector3(1,0,0));
	  
	  btVector3 unit = btVector3(1,0,0);
	  unit = rotation*unit;
	  double angle_between = btDot(force[0],unit);
	  simlog << worldTime - dt << "," << angle_between/force[0].norm()/unit.norm() << "," <<
	    phi << "," << theta << ",\n"; // write thrust to file
	  
	}
      */
  
    }
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
