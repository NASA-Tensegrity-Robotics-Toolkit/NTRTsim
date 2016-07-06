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
  double sf = 30; // scaling factor
  double worldTime = 0.0; // clock for world time (UPDATED: clock for time thruster controller is on)
  double shootTime = 0.0; // used for thrust timing in multiple hop scenarios
  double initiateThrustTime = 3; // wait time until thrust initiation
  double reorientTime = initiateThrustTime + 0;
  bool isReoriented = false;
  double thrustPeriod = 100; // duration of thrust on
  bool thrusted = false; // records if this cycle of thrust has happened or not for each hop
  double targetDistance = 10.0; // distance to the target (before scaling)
  const btVector3 targetLocation = btVector3(targetDistance*sf, 0.0, 0.0); // target is located 1000m away in +X direction	
  bool doneHopping = false; // end hopping when this is true
  int numberOfHops = 0; // count the number of hops the robot made in this simulation	
  std::ofstream simlog; // log file for thruster related variables
  bool doLog = false; // choose to log thruster related variables
  double timePassed = 0.0; // controls logging frequency
  bool includeNoise = false; //turn on/off Gaussian Noise for thruster orientation
  double prev_alpha = 0;
  double prev_beta = 0;
  double prev_gimbalYaw = 0;
  double prev_gimbalPitch = 0;
  btVector3 prev_GimbalHeading = btVector3(0,0,-1);
  int numTankOrientations = 1; //Averaging Filter
  int count = 0;
  
  //##### BCera - moved from onStep to namespace, to initialize jetDirections onSetup
  double thrust = 50*sf;//50*sf;
  //double thrust = 0;
  //double final_thrust = 100*sf;

  btRigidBody* tankRigidBody;
  btRigidBody* thrusterRigidBody; // payload body
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

  tankRigidBody = subject.TankBodies[0];
  thrusterRigidBody = subject.ThrusterBodies[0]; // payload body

  tankOrientations.resize(numTankOrientations);

  prev_angle_err = 0;
  error_sum = 0;

  //Set Initial Launch Orientation Here ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  goalAltitude = -90;
  goalYaw = 0;
  goalVector.setX(sin((goalAltitude+90)*M_PI/180)*sin(goalYaw*M_PI/180));
  goalVector.setY(cos((goalAltitude+90)*M_PI/180));
  goalVector.setZ(sin((goalAltitude+90)*M_PI/180)*cos(goalYaw*M_PI/180));
  goalVector = goalVector.normalized();
  //goalVector = goalVector.rotate(btVector3(1,0,0),M_PI); // Rotate goalVector 180 degrees! To match Ed's walking controller
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


  //tankRigidBody = subject.TankBodies[0];
  //thrusterRigidBody = subject.ThrusterBodies[0]; // payload body

  
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
  
  if(doLog){
    sim_out.open("Payload Thruster Control Vertical - Locked ",std::ios::app);
    sim_out << "Label-Goal Altitude, GoalAltitude, Label-GoalYaw, GoalYaw, Label-SimTime, SimTime, Label-GimbalPitch, GimbalPitch, Label-GimbalYaw, GimbalYaw, Label-TankPitch, TankPitch, Label-TankYaw, TankYaw, Label-TankPos, TankPosX, TankPosY, TankPosZ, Label-TankVelPitch, TankVelPitch, Label-TankVelYaw, TankVelYaw, Label-Alpha, Alpha, Label-Beta, Beta, Label-Error, Error, Label-d_error, d_Error" << std::endl;
    sim_out << std::endl;
  }
}


void RPThruster::onStep(PrismModel& subject, double dt)
{
  if (dt <= 0.0)
    {
      throw std::invalid_argument("dt is not positive");
    }
  else if(subject.robotState==2)
    {
      //Thruster Orientation=============================================================================================

      // Precondition
      if (dt <= 0.0)
	{
	  throw std::invalid_argument("dt is not positive");
	}
      else
	{
	  if(worldTime > reorientTime && !isReoriented){
	    std::cout << "Reoriented Thrust" << std::endl;
	    double finalGoalAltitude = -10;
	    double finalGoalYaw = 45;
	    double inc = 40*dt;
	    bool AltSet = false;
	    bool YawSet = false;
	    if(abs(finalGoalAltitude - goalAltitude) > inc){
	      if(finalGoalAltitude-goalAltitude < 0)
		goalAltitude -= inc;
	      if(finalGoalAltitude-goalAltitude > 0)
		goalAltitude += inc;
	    }
	    else{
	      goalAltitude += (finalGoalAltitude-goalAltitude);
	      AltSet = true;
	    }
	    
	    if(abs(finalGoalYaw - goalYaw) > inc){
	      if(finalGoalYaw-goalYaw < 0)
		goalYaw -= inc;
	      if(finalGoalYaw-goalYaw > 0)
		goalYaw += inc;
	    }
	    else{	           
	      goalYaw += (finalGoalYaw-goalYaw);
	      YawSet = true;
	    }
	    
	    //Find updated goalVector ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	    goalVector.setX(sin((goalAltitude+90)*M_PI/180)*sin(goalYaw*M_PI/180));
	    goalVector.setY(cos((goalAltitude+90)*M_PI/180));
	    goalVector.setZ(sin((goalAltitude+90)*M_PI/180)*cos(goalYaw*M_PI/180));
	    goalVector = goalVector.normalized();
	    //goalVector = goalVector.rotate(btVector3(1,0,0),M_PI); // Rotate goalVector 180 degrees! To match Ed's walking controller
	    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	    
	    if(AltSet && YawSet)
	      isReoriented = true;
	  }
	  
	  std::cout << "Simulation Time: " << worldTime << std::endl;
	  std::cout << "Goal Altitude: " << goalAltitude << "; Goal Yaw: " << goalYaw << std::endl;
	  double altitudeAngle = subject.altitudeHinge->getHingeAngle()*180.0/M_PI;
	  double yawAngle = subject.yawHinge->getHingeAngle()*180.0/M_PI;
	  std::cout << "Goal Vector: " << goalVector << std::endl;
	  std::cout << "Gimbal Inputs || Altitude Angle: " << altitudeAngle << ", Yaw Angle: " << yawAngle << std::endl;
	  std::cout << "Gimbal Speeds || Altitude Speed: " << (altitudeAngle-prev_gimbalPitch)/dt << ", Yaw Speed: " << (yawAngle-prev_gimbalYaw)/dt << std::endl;
	  prev_gimbalPitch = altitudeAngle;
	  prev_gimbalYaw = yawAngle;

	  //std::cout << "Altitude angle: " << altitudeAngle << std::endl;
	  //std::cout << "Yaw angle: " << yawAngle << std::endl;

	  bool altitudeInPosition = false;
	  bool yawInPosition = false;
	  double altitude_corr;
	  double yaw_corr;

	  //Corrected Goal Altitude
	  btTransform world_corr_mat = tankRigidBody->getWorldTransform();
	  btMatrix3x3 world_rot_mat = world_corr_mat.getBasis();
	  btVector3 unit_tank = btVector3(0,1,0);
	  unit_tank = world_rot_mat*unit_tank;
	  unit_tank = unit_tank.normalized();
	  /*
	  //Average Filter#########################################################################################
	  tankOrientations[count%numTankOrientations] = unit_tank;
	  count++;
	  unit_tank = btVector3(0,0,0);
	  for(int i=0;i<numTankOrientations;i++){
	    unit_tank.setX(unit_tank.getX() + tankOrientations[i].getX());
	    unit_tank.setY(unit_tank.getY() + tankOrientations[i].getY());
	    unit_tank.setZ(unit_tank.getZ() + tankOrientations[i].getZ());
	  }
	  unit_tank.setX(unit_tank.getX()/numTankOrientations);
	  unit_tank.setY(unit_tank.getY()/numTankOrientations);
	  unit_tank.setZ(unit_tank.getZ()/numTankOrientations);
	  //########################################################################################################
	  */
	  std::cout << "Tank Heading|| Altitude: " << acos(unit_tank.y())*180/M_PI-90 << ", Yaw:  " << atan2(unit_tank.z(),-unit_tank.x())*180/M_PI-90 << std::endl;
	  btVector3 tank_pos = tankRigidBody->getCenterOfMassPosition();
	  std::cout << "Tank Position: " << tank_pos << std::endl;
	  btVector3 tank_lin_vel = tankRigidBody->getLinearVelocity();
	  std::cout << "Tank Velocity Magnitude: " << tank_lin_vel.length() << std::endl;
	  tank_lin_vel = tank_lin_vel.normalized();
	  std::cout << "Tank Velocity Direction|| Altitude: " << acos(tank_lin_vel.y())*180/M_PI-90 << ", Yaw:  " << atan2(tank_lin_vel.z(),-tank_lin_vel.x())*180/M_PI-90 << std::endl;
	  btVector3  sol_vector = world_rot_mat.transpose()*goalVector; //P_p
	  std::cout << "solution vector" << sol_vector <<  std::endl;
	  sol_vector.setX(-sol_vector.getX());
	  sol_vector.setZ(-sol_vector.getZ()); //Reoriented goal vector for thruster

	  //Set PID Controller Here
	  //NO-ROBOT CONSTANTS
	  /*
	  double Kp = 0.005;//0.001;
	  double Kd = 5;//1*(.001/dt);
	  double Ki = 5e-7*(dt/.001);
	  */
	  //WITH-ROBOT CONSTANTS
	  /*
	  double Kp = .08;
	  double Kd = 275;//5*(.001/dt);
	  double Ki = 1e-5;//1e-7*(dt/.001);
	  */
	  
	  btVector3 PayloadYAxis = -btVector3(world_rot_mat[0][1],world_rot_mat[1][1],world_rot_mat[2][1]);
	  //PayloadYAxis is negative, because frames are not lined up during setup (180 rotation)
	  double angle_error = acos(PayloadYAxis.dot(goalVector));
	  double d_angle_error = (angle_error-prev_angle_err)/dt;
	  prev_angle_err = angle_error;

	  /*
	  //PID Controller
	  error_sum += angle_error;
	  if(angle_error<5*M_PI/180)
	    error_sum = 0;
	  sol_vector.setY(0);
	  sol_vector = sol_vector.normalized(); //unit directional vector on the X-Z plane
	  double scalingFactor = (Kp*angle_error + Kd*d_angle_error*dt + Ki*error_sum);
	  std::cout << "Scaling Factor: " << scalingFactor << ", error: " << angle_error*180/M_PI << ", d_error: " << d_angle_error*dt << ", i_error: " << error_sum << std::endl;
	  sol_vector.setX(scalingFactor*sol_vector.getX());
	  sol_vector.setZ(scalingFactor*sol_vector.getZ()); //Reoriented goal vector for thruster
	  sol_vector.setY(0.25);
	  sol_vector = sol_vector.normalized();
	  //std::cout << "Normalized Solution Vector: " << sol_vector << std::endl;
	  */

	  //LQR Controller
	  double des_moment = (5.4678*(angle_error) + 3.4543*(d_angle_error)); //discrete LQR with Q=[30 0; 0 1], R=1
	  //double des_moment = (5.4682*(angle_error) + 3.3070*(d_angle_error)); //discrete LQR with Q=[30 0; 0 0], R=1
	  des_moment = des_moment*200;//200;//542.4; //Uncomment this line for with-robot control
	  std::cout << "des_moment: " << des_moment << std::endl;
	  if(des_moment < 0){
	    sol_vector.setX(-sol_vector.getX());
	    sol_vector.setZ(-sol_vector.getZ()); //Reoriented goal vector for thruster
	    des_moment = -des_moment;
	  }
	  double theta = des_moment/(0.055*sf+0.05*sf+0.025*sf)/thrust; //externalRadius+tanktogimbal+payloadlength/2
	  if(des_moment > (0.055*sf+0.05*sf+0.025*sf)*thrust)
	    theta = M_PI/2;
	  else
	    theta = asin(theta);
	  sol_vector.setY(sqrt(pow(sol_vector.getX(),2)+pow(sol_vector.getZ(),2))/tan(theta));
	  sol_vector.normalize();
	  double Gimbal_Speed = acos(sol_vector.dot(prev_GimbalHeading))/dt;
	  prev_GimbalHeading = sol_vector;
	  std::cout << "Gimbal Change Speed: " << Gimbal_Speed*180/M_PI << std::endl;
	  //Generate noise for solution vector here (i.e. noisy orientation sensors for payload)
	  if(includeNoise){
	    btTransform noiseRotation;
	    btQuaternion rotation;
	    btVector3 noiseAxis;
	    noiseAxis.setX(rand());
	    noiseAxis.setY(rand());
	    noiseAxis.setZ(rand());
	    noiseAxis.normalize();
	    rotation.setRotation(noiseAxis, generateGaussianNoise(0,2*M_PI/180));
	    noiseRotation.setRotation(rotation);
	    std::cout << "Sensor Noise Error Introduced: " << rotation.getAngle()*180/M_PI << std::endl;
	    sol_vector = noiseRotation*sol_vector;
	  }
	  std::cout << "Thruster Goal Vector: " << sol_vector << std::endl;
	  std::cout << "error: " << angle_error*180/M_PI << ", d_error: " << d_angle_error*180/M_PI << ", i_error: " << error_sum*180/M_PI << std::endl;
	  
	  //Calculate Gimbal Inputs
	  double beta = asin(-sol_vector.getY());
	  double alpha;
	  alpha = atan2(sol_vector.getX(),sol_vector.getZ());
	  //Gimbal Limits
	  if(beta<-M_PI/2-M_PI/4)
	    beta = -M_PI/2-M_PI/4;
	  if(beta>-M_PI/2+M_PI/4)
	    beta = -M_PI/2+M_PI/4;
	  beta = -beta;
	  //alpha = -alpha;
	  prev_alpha = alpha;
	  prev_beta = beta;
	  std::cout << "Alpha: " << alpha*180/M_PI << "; Beta: " << beta*180/M_PI << std::endl;
	  std::cout << "Alpha Speed: " << (alpha-prev_alpha)*180/M_PI/dt << "; Beta Speed: " << (beta-prev_beta)*180/M_PI/dt << std::endl;


	  double tolerance = 1; //Degree
	  double speed = 3; //Degrees - No scaling factor here (?)
	  
	  //Set altitude
	  double deltaAltitude = beta*180/M_PI - altitudeAngle;
	  if (deltaAltitude > tolerance){
	    subject.altitudeHinge->enableAngularMotor(true,deltaAltitude*speed,1);
	    //subject.altitudeHinge->enableAngularMotor(true,speed,1e4);
	    //subject.altitudeHinge->setMotorTarget((btScalar)beta,dt);
	    //subject.altitudeHinge->enableMotor(true);
	  }
	  else if (deltaAltitude < -tolerance){
	    subject.altitudeHinge->enableAngularMotor(true,deltaAltitude*speed,1);
	    //subject.altitudeHinge->enableAngularMotor(true,-speed,1e4);
	    //subject.altitudeHinge->setMotorTarget((btScalar)beta,dt);
	    //subject.altitudeHinge->enableMotor(true);
	  }
	  else{
	    subject.altitudeHinge->enableAngularMotor(true,0.0,10);
	    altitudeInPosition = true;
	  }

	  //Set yaw
	  double deltaYaw = alpha*180/M_PI - yawAngle;
	  if (deltaYaw-speed*dt > tolerance){
	    subject.yawHinge->enableAngularMotor(true,deltaYaw*speed,1);
	    //subject.yawHinge->enableAngularMotor(true,speed,10);
	    //subject.yawHinge->setMotorTarget((btScalar)alpha,dt);
	    //subject.yawHinge->enableMotor(true);
	  }
	  else if (deltaYaw+speed*dt < -tolerance){
	    subject.yawHinge->enableAngularMotor(true,deltaYaw*speed,1);
	    //subject.yawHinge->enableAngularMotor(true,-speed,10);
	    //subject.yawHinge->setMotorTarget((btScalar)alpha,dt);
	    //subject.yawHinge->enableMotor(true);
	  }
	  else{
	    subject.yawHinge->enableAngularMotor(true,0.0,1);
	    yawInPosition = true;
	  }

	  std::cout << "[ ] Yaw error " << deltaYaw << " Altitude error " << deltaAltitude << std::endl;

	  //Actual Thruster Heading
	  //btRigidBody* thrusterRigidBody = subject.ThrusterBodies[0]; // payload body
	  btMatrix3x3 rotation = thrusterRigidBody->getWorldTransform().getBasis();
	  btVector3 unit = btVector3(0,0,1);
	  unit = rotation*unit;
	  
	  //std::cout << "Thruster Heading: " << unit.x() << " " << unit.y() << " " << unit.z() << std::endl << std::endl;;
	  std::cout << "Thruster Heading|| Altitude: " << acos(unit.y())*180/M_PI-90 << ", Yaw:  " << atan2(unit.z(),-unit.x())*180/M_PI-90 << std::endl;

	  //Thruster Force Vector===========================================================================================
	  
	  //std::vector<tgRod *> thrusterParts = PrismModel::find<tgRod>("thruster");
	  //Get thruster transform
	  //tgRod* thrusterRod = thrusterParts[0]; //Outer
	  //btRigidBody* thrusterRigidBody = thrusterRod->getPRigidBody();
	  
	  std::cout << "Thrust: " << thrust << std::endl;
	  double gimbalAngleTol = 200;
	  if ( shootTime > initiateThrustTime && shootTime <= (initiateThrustTime + thrustPeriod) && abs(deltaAltitude)<gimbalAngleTol && abs(deltaYaw)<gimbalAngleTol)
	    {
	      std::cout << "Thruster Active~" << std::endl;
	      for(int k=0; k<force.size(); k++)
		{
		  force[k] = thrust/jetnumber*jetDirections[k]; // assume target is towards +X direction
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
	  std::cout << std::endl;
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

	  std::cout << fmod(worldTime,0.1) << std::endl;
	  if(doLog && fmod(worldTime,0.1)<0.001){
	    std::cout << "PRINTING TO FILE~~~~~"  << std::endl;
	    sim_out << "Goal Altitude, " << goalAltitude << ", Goal Yaw, " << goalYaw << ", ";
	    sim_out << "Sim Time, " << worldTime << ", Gimbal Pitch, " << altitudeAngle << ", Gimbal Yaw, " << yawAngle << ", ";
	    sim_out << "Tank Pitch, " << acos(unit_tank.y())*180/M_PI-90 << ", Tank Yaw, " << atan2(unit_tank.z(),-unit_tank.x())*180/M_PI-90 << ", ";
	    sim_out << "Tank Pos, " << tank_pos[0] << ", " << tank_pos[1] << ", " << tank_pos[2] << ", ";
	    sim_out << "Tank Vel Pitch, " << acos(tank_lin_vel.y())*180/M_PI-90 << ", Tank Vel Yaw,  " << atan2(tank_lin_vel.z(),-tank_lin_vel.x())*180/M_PI-90 << ", ";
	    sim_out << "Alpha, " << alpha*180/M_PI << ", Beta, " << beta*180/M_PI << ", ";
	    //sim_out << "Scaling Factor, " << scalingFactor << ", error, " << angle_error*180/M_PI << ", d_error, " << d_angle_error << ", i_error, " << error_sum << std::endl;
	    sim_out << "Error, " << angle_error*180/M_PI << ", d_Error, " << d_angle_error << std::endl;
	    sim_out << std::endl;
	  }

	  if(worldTime > 30 && doLog){
	    doLog = false;
	    sim_out.close();
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
    const double two_pi = 2*M_PI;

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
