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
#include "RPModel.h"
// This library
#include "core/abstractMarker.h" //##### BCera - Added for later use of abstract markers
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
  const int configuration = 2; // 1 - sit on open base, 2 - sit on closed base
  const double sf = 3;//scaling factor with respect to meter scale. E.g., centimeter scale is achieved by setting sf = 100
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
	214.8/pow(sf,3),//0.688,    // density (kg / length^3)
	0.0143*sf,//0.31,     // radius (length)
	400,//1192.5*10,//613.0,   // stiffness (kg / sec^2) was 1500
	500,    // damping (kg / sec)
	1.5*sf, //.6858*sf,     // rod_length (length)
	.05*sf,      // rod_space (length)
	0.99,      // friction (unitless)
	0.1,     // rollFriction (unitless)
	0.0,      // restitution (?)
	500*sf, //610,        // pretension -> set to 4 * 613, the previous value of the rest length controller
	0,			// History logging (boolean)
	50*sf,   // maxTens
	0.5*sf,//1, //sf,    // targetVelocity

	// Use the below values for earlier versions of simulation.
	// 1.006,    
	// 0.31,     
	// 300000.0, 
	// 3000.0,   
	// 15.0,     
	// 7.5,      
      };


  // Payload rod properties
  double payloadLength = .25*sf;//0.15*sf; // [payload] length, shortest edge is 7cm (7cm x 12cm x 11cm)
  double payloadRadius = 0.0648*sf; // [payload] radius (in meters)  
  double payloadWeight = 10.0 - 0.09463*6; // [payload] weight (kg) -- Thruster robot  
  double payloadDensity = payloadWeight/(0.11*0.12*0.07)/pow(sf,3);

  // helper arrays for node and rod numbering schema
  // returns the number of the rod for a given node
  const int rodNumbersPerNode[12]={0,0,1,1,2,2,3,3,4,4,5,5};

} // namespace

RPModel::RPModel() : tgModel() 
{
}

RPModel::~RPModel()
{
  std::size_t n = markers.size();
  for(std::size_t i = 0; i < n; i++)
    {
      delete &markers[i];
    }
  markers.clear();
  allAbstractMarkers.clear();
  allActuators.clear();
  allNodes.clear();
  allRods.clear();
  allBaseRigids.clear();
  nodePositions.clear();
}

void RPModel::addNodes(tgStructure& s)
{
  
  // initial node positions for six rods from Kyunam's original code	
  /*
  s.addNode(-1.87032*sf/10,   0.613512*sf/10,  -6.15187*sf/10);  // 0
  s.addNode(0.265642*sf/10,   2.47087*sf/10,   -4.83741*sf/10);  // 1
  s.addNode(-2.23617*sf/10,   4.444*sf/10,     -7.67582*sf/10);  // 2
  s.addNode(-5.74677*sf/10,   2.20731*sf/10,   -6.89138*sf/10);  // 3
  s.addNode(-1.08779*sf/10,   6.32123*sf/10,   -3.99281*sf/10);  // 4
  s.addNode(-4.60576*sf/10,   6.21171*sf/10,   -6.2681*sf/10);   // 5
  s.addNode(-6.84092*sf/10,   4.4103*sf/10,    -3.33944*sf/10);  // 6
  s.addNode(-4.74128*sf/10,   6.21108*sf/10,   -2.01362*sf/10);  // 7
  s.addNode(-2.19455*sf/10,   0.613006*sf/10,  -1.98777*sf/10);  // 8
  s.addNode(-5.64078*sf/10,   0.611434*sf/10,  -4.40176*sf/10);  // 9
  s.addNode(-0.920689*sf/10,  4.54442*sf/10,   -1.35471*sf/10);  // 10
  s.addNode(-4.39565*sf/10,   2.26477*sf/10,   -0.663673*sf/10); // 11
  */

  //Initial Node Positions for Six Rods, Matched to MATLAB Code ################################################

  double L = c.rod_length;
  double t = 1.2213*L/1.9912; //find exact geometric ratio later
  double r = t/(2*sin(36*M_PI/180)); 
  double theta = asin(r/t)*180/M_PI;
  double m = sqrt(pow(t,2)+pow((2*t*sin(54*M_PI/180)),2)); //tip to tip of reg. icosahedron
  double var = m-t*cos(theta*M_PI/180);

  //Values below copied from Icosahderon_w_payload.m, with Y_ntrt = Z_matlab  and Z_ntrt = -Y_matlab
  //Coordinate transformation necessary as NTRT height direction is (0,1,0) Y unit vector
  
  s.addNode(0, 0, 0);  // 0
  s.addNode(0, var*sf, r*sf);  // 1
  s.addNode(0, t*cos(theta*M_PI/180)*sf, -r*sf);  // 2
  s.addNode(0, m*sf, 0*sf); // 3
  s.addNode(-r*cos(18*M_PI/180)*sf,   t*cos(theta*M_PI/180)*sf,    -r*sin(18*M_PI/180)*sf);   // 4
  s.addNode(r*cos(18*M_PI/180)*sf,   t*cos(theta*M_PI/180)*sf,    -r*sin(18*M_PI/180)*sf);  // 5
  s.addNode(-r*cos(18*M_PI/180)*sf,   var*sf,    r*sin(18*M_PI/180)*sf);  // 6
  s.addNode(r*cos(18*M_PI/180)*sf,   var*sf,    r*sin(18*M_PI/180)*sf);  // 7
  s.addNode(-r*sin(36*M_PI/180)*sf,   t*cos(theta*M_PI/180)*sf,    r*cos(36*M_PI/180)*sf);  // 8
  s.addNode(-r*sin(36*M_PI/180)*sf,   var*sf,    -r*cos(36*M_PI/180)*sf);  // 9
  s.addNode(r*sin(36*M_PI/180)*sf,   t*cos(theta*M_PI/180)*sf,    r*cos(36*M_PI/180)*sf);  // 10
  s.addNode(r*sin(36*M_PI/180)*sf,   var*sf,    -r*cos(36*M_PI/180)*sf);  // 11
  /*
  s.addNode(0, 0, 0);  // 0
  s.addNode(0, var*sf/10, r*sf/10);  // 1
  s.addNode(0, t*cos(theta*M_PI/180)*sf/10, -r*sf/10);  // 2
  s.addNode(0, m*sf/10, 0*sf/10); // 3
  s.addNode(-r*cos(18*M_PI/180)*sf/10,   t*cos(theta*M_PI/180)*sf/10,    -r*sin(18*M_PI/180)*sf/10);   // 4
  s.addNode(r*cos(18*M_PI/180)*sf/10,   t*cos(theta*M_PI/180)*sf/10,    -r*sin(18*M_PI/180)*sf/10);  // 5
  s.addNode(-r*cos(18*M_PI/180)*sf/10,   var*sf/10,    r*sin(18*M_PI/180)*sf/10);  // 6
  s.addNode(r*cos(18*M_PI/180)*sf/10,   var*sf/10,    r*sin(18*M_PI/180)*sf/10);  // 7
  s.addNode(-r*sin(36*M_PI/180)*sf/10,   t*cos(theta*M_PI/180)*sf/10,    r*cos(36*M_PI/180)*sf/10);  // 8
  s.addNode(-r*sin(36*M_PI/180)*sf/10,   var*sf/10,    -r*cos(36*M_PI/180)*sf/10);  // 9
  s.addNode(r*sin(36*M_PI/180)*sf/10,   t*cos(theta*M_PI/180)*sf/10,    r*cos(36*M_PI/180)*sf/10);  // 10
  s.addNode(r*sin(36*M_PI/180)*sf/10,   var*sf/10,    -r*cos(36*M_PI/180)*sf/10);  // 11
  */
  /*
  // axis conventions kept same as MATLAB
  s.addNode(0, 0, 0);  // 0
  s.addNode(0, -r*sf/10, var*sf/10);  // 1
  s.addNode(0, r*sf/10, t*cos(theta*M_PI/180)*sf/10);  // 2
  s.addNode(0, 0*sf/10, m*sf/10); // 3
  s.addNode(-r*cos(18*M_PI/180)*sf/10,    r*sin(18*M_PI/180)*sf/10,   t*cos(theta*M_PI/180)*sf/10);   // 4
  s.addNode(r*cos(18*M_PI/180)*sf/10,    r*sin(18*M_PI/180)*sf/10,   t*cos(theta*M_PI/180)*sf/10);  // 5
  s.addNode(-r*cos(18*M_PI/180)*sf/10,   -r*sin(18*M_PI/180)*sf/10,   var*sf/10);  // 6
  s.addNode(r*cos(18*M_PI/180)*sf/10,    -r*sin(18*M_PI/180)*sf/10,   var*sf/10);  // 7
  s.addNode(-r*sin(36*M_PI/180)*sf/10,   -r*cos(36*M_PI/180)*sf/10,   t*cos(theta*M_PI/180)*sf/10);  // 8
  s.addNode(-r*sin(36*M_PI/180)*sf/10,    r*cos(36*M_PI/180)*sf/10,   var*sf/10);  // 9
  s.addNode(r*sin(36*M_PI/180)*sf/10,    -r*cos(36*M_PI/180)*sf/10,   t*cos(theta*M_PI/180)*sf/10);  // 10
  s.addNode(r*sin(36*M_PI/180)*sf/10,    r*cos(36*M_PI/180)*sf/10,   var*sf/10);  // 11
  */
  //#############################################################################################################

  // get 12 rod end nodes
  tgNodes rodNodes = s.getNodes();    
    
  // define payload nodes
  double rodNodesCOM[3] = {0.0, 0.0, 0.0};
  for ( size_t i = 0; i < 12; i++ )
    {
      rodNodesCOM[0] += rodNodes[i][0];
      rodNodesCOM[1] += rodNodes[i][1];
      rodNodesCOM[2] += rodNodes[i][2];
    }
  rodNodesCOM[0] /= 12;
  rodNodesCOM[1] /= 12;
  rodNodesCOM[2] /= 12;

  // node # 12,13: using initial COM
  L = payloadLength;
  double p_theta = .314113413512790;
  double p_phi = .652358139784368;
  s.addNode(rodNodesCOM[0]+L/2*cos(p_theta)*sin(p_phi), rodNodesCOM[1]+L/2*cos(p_phi), rodNodesCOM[2]-L/2*sin(p_theta)*sin(p_phi)); // 12 , length of EV3 is 7 cm
  s.addNode(rodNodesCOM[0]-L/2*cos(p_theta)*sin(p_phi), rodNodesCOM[1]-L/2*cos(p_phi), rodNodesCOM[2]+L/2*sin(p_theta)*sin(p_phi)); // 13	 
	
  // Create an array of node positions to attach markers later
  tgNodes allNodes = s.getNodes();  
	    
  for (int i = 0; i < allNodes.size(); i++)
    {
      nodePositions.push_back(btVector3(allNodes[i]));				
    }
}

void RPModel::addRods(tgStructure& s)
{
  // structure rods (lower number has darker marker)
  s.addPair( 0,  1, "rod");
  s.addPair( 2,  3, "rod");
  s.addPair( 4,  5, "rod");
  s.addPair( 6,  7, "rod");
  s.addPair( 8,  9, "rod");
  s.addPair( 10, 11, "rod");

  // payload
  s.addPair(12, 13, "payload");
}

void RPModel::addActuators(tgStructure& s)
{
  // be careful with ordering of nodes, numbers match connectivity matrix in MATLAB file icosahedron_w_payload.m
  // outer cables	
  s.addPair( 0, 4,  "muscle");	//0	
  s.addPair( 0, 5,  "muscle");	//1
  s.addPair( 0, 8,  "muscle");	//2
  s.addPair( 0, 10, "muscle");	//3	
  s.addPair( 1, 8,  "muscle");	//4
  s.addPair( 1, 10, "muscle");	//5
  s.addPair( 1, 6,  "muscle");	//6
  s.addPair( 1, 7,  "muscle");	//7
  s.addPair( 2, 4,  "muscle");	//8
  s.addPair( 2, 5,  "muscle");	//9
  s.addPair( 2, 9,  "muscle");	//10
  s.addPair( 2, 11, "muscle");	//11
  s.addPair( 3, 6,  "muscle");	//12
  s.addPair( 3, 7,  "muscle");	//13
  s.addPair( 3, 9,  "muscle");	//14
  s.addPair( 3, 11, "muscle");	//15
  s.addPair( 4, 8,  "muscle");	//16
  s.addPair( 4, 9,  "muscle");	//17
  s.addPair( 5, 10, "muscle");	//18
  s.addPair( 5, 11, "muscle");	//19
  s.addPair( 6, 8,  "muscle");	//20
  s.addPair( 6, 9,  "muscle");	//21
  s.addPair( 7, 10, "muscle");	//22
  s.addPair( 7, 11, "muscle");  //23

  // Adding payload connection
  // inner cables
  s.addPair( 0,  13, "payload_muscle"); //24
  s.addPair( 4,  13, "payload_muscle"); //25
  s.addPair( 8,  13, "payload_muscle"); //26
  s.addPair( 3, 12, "payload_muscle"); //27
  s.addPair( 7,  12, "payload_muscle"); //28
  s.addPair( 11,  12, "payload_muscle"); //29
}

void RPModel::addMarkers(tgStructure& s)
{			
  std::vector<tgRod *> rods = find<tgRod>("rod"); // name tag follows what is used in tgBuildSpec	
	
  // Attach markers to the first 12 nodes corresponding to rod ends
  for(unsigned i = 0; i < 2*rods.size(); i++)
    {
      const btRigidBody* bt = rods[rodNumbersPerNode[i]]->getPRigidBody();
      btTransform inverseTransform = bt->getWorldTransform().inverse(); //inverse T is world to model!
      btVector3 pos = inverseTransform * (nodePositions[i]);
      abstractMarker tmp=abstractMarker(bt,pos,btVector3(1.0/12.0*i,1.0/12.0*i,1.0/12.0*i),i); // body, position, color, node number
      this->addMarker(tmp);
    }
	
  // Attach a marker at the center of the payload
  std::vector<tgRod *> payload = find<tgRod>("payload");
  const btRigidBody* bt = payload[0]->getPRigidBody();
  btTransform inverseTransform = bt->getWorldTransform().inverse();
  btVector3 payloadNode1 = nodePositions[nodePositions.size()-2];
  btVector3 payloadNode2 = nodePositions[nodePositions.size()-1];
  btVector3 payloadCenterNode = (payloadNode1 + payloadNode2) / 2.0;
  btVector3 thrustDir = btVector3(0,7,0);	
  btVector3 pos = inverseTransform * (payloadCenterNode);
  btVector3 pos2 = (thrustDir);
  abstractMarker tmp=abstractMarker(bt,pos,btVector3(.0,.0,.0),0); // body, position, color, node number
  //this->addMarker(tmp); //removed marker at payload center
  abstractMarker tmp2=abstractMarker(bt,pos2,btVector3(4.5,.0,.0),13); // body, position, color, node number
  this->addMarker(tmp2); //added thruster orientation marker
  
}

// Created function to detect and edit current bottom nodes
void RPModel::findBottomNodes(){
  markers = this -> getMarkers();
  abstractMarker m1=markers[0];
  abstractMarker m2=markers[1];

  double minM=std::min(m1.getWorldPosition().getY(),m2.getWorldPosition().getY());
  if (m1.getWorldPosition().getY()==minM){
    mGrnd[0]=0;
  }
  else {
    mGrnd[0]=1;
  }
  for (std::size_t i=2; i<markers.size(); i++){
    abstractMarker mTemp=markers[i];
    double  minM1=std::min(markers[mGrnd[0]].getWorldPosition().getY(),mTemp.getWorldPosition().getY());
    if (mTemp.getWorldPosition().getY()==minM1){
      mGrnd[0]=i;
    }
  }
  int start = 0;
  if(start==mGrnd[0]){
    start++;
  }
  mGrnd[1] = start;
  for (std::size_t i=0; i<markers.size(); i++){
    abstractMarker mTemp=markers[i];
    double minM2=std::min(markers[mGrnd[1]].getWorldPosition().getY(),mTemp.getWorldPosition().getY());
    if (mTemp.getWorldPosition().getY()==minM2 && i!=mGrnd[0]){
      mGrnd[1]=i;
    }
  }
  start = 0;
  if(start==mGrnd[0] || start==mGrnd[1]){
    start++;
  }
  mGrnd[2] = start;
  for (std::size_t i=0; i<markers.size(); i++){
    abstractMarker mTemp=markers[i];
    double minM3=std::min(markers[mGrnd[2]].getWorldPosition().getY(),mTemp.getWorldPosition().getY());
    if (mTemp.getWorldPosition().getY()==minM3 && i!=mGrnd[0] && i!=mGrnd[1]){
      mGrnd[2] = i;
    }
  }
}

void RPModel::setup(tgWorld& world)
{

  markers.clear();

  const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);

  //Additional config for added center payload
  const tgRod::Config payloadConfig(payloadRadius, payloadDensity, c.friction, 
				    c.rollFriction, c.restitution);	
    
  /// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate
  tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension, c.hist, 
				       c.maxTens, c.targetVelocity);
            
  // Start creating the structure
  tgStructure s;
  addNodes(s);
  addRods(s);
  addActuators(s);

  btTransform correction;

  
  //###########################################################################################################################################
  // Add a rotation. This is needed if the ground slopes too much,
  // otherwise  glitches put a rod below the ground.

  double L = c.rod_length;
  double t = 1.2213*L/1.9912; //find exact geometric ratio later
  double r = t/(2*sin(36*M_PI/180)); 
  double theta = asin(r/t);

  if(configuration == 1){ // OPEN BASE CONFIGURATION
  
    btVector3 rotationPoint = btVector3(0, 0, 0); // origin

    /*
    //180 flip
    btVector3 rotationAxis_flip = btVector3(0, 0, 1); 
    double rotationAngle_flip = M_PI/2;
    btQuaternion rot_corr_flip(rotationAxis_flip,rotationAngle_flip);
    s.addRotation(rotationPoint, rotationAxis_flip, rotationAngle_flip);
    */

    btVector3 rotationAxis_Y = btVector3(0, 1, 0); 
    double rotationAngle_Y = M_PI/4;
    btQuaternion rot_corr_Y(rotationAxis_Y,-rotationAngle_Y); //LEFT HAND ROTATION CONVENTION IN NTRT WHYYYYYYYYYYYYYYYYYYYYYYYYYY
    btVector3 rotationAxis_X = btVector3(1, 0, 0); 
    double rotationAngle_X = -asin(2*cos(theta)/sqrt(3));
    btQuaternion rot_corr_X(rotationAxis_X,-rotationAngle_X);

    s.addRotation(rotationPoint, rotationAxis_X, -rotationAngle_X);
    s.addRotation(rotationPoint, rotationAxis_Y, -rotationAngle_Y);

    
    correction.setRotation(rot_corr_Y*rot_corr_X);

    btVector3 translate = btVector3(0, 4, 0); //relative to rotated frame
    s.move(translate);  

    for(unsigned i = 0; i < nodePositions.size(); i++){
      nodePositions[i] = correction*nodePositions[i]; //correct marker positions
      nodePositions[i] = translate+nodePositions[i]; //correct marker positions
    }
    
  }

  if(configuration == 2){ // CLOSED BASE CONFIGURATION
    btVector3 rotationPoint = btVector3(0, 0, 0); // origin

    btVector3 rotationAxis_Z = btVector3(0, 0, 1); 
    double rotationAngle_Z = -asin(2*cos(theta)/sqrt(3));
    btQuaternion rot_corr_Z(rotationAxis_Z,-rotationAngle_Z);
    btVector3 rotationAxis_Y = btVector3(0, 1, 0); 
    double rotationAngle_Y = 18*M_PI/180;
    btQuaternion rot_corr_Y(rotationAxis_Y,-rotationAngle_Y);

    s.addRotation(rotationPoint, rotationAxis_Z, -rotationAngle_Z);
    s.addRotation(rotationPoint, rotationAxis_Y, -rotationAngle_Y);

    correction.setRotation(rot_corr_Y*rot_corr_Z);

    btVector3 translate = btVector3(0, 10, 0); //relative to rotated frame
    s.move(translate);  

      
    for(unsigned i = 0; i < nodePositions.size(); i++){
      nodePositions[i] = correction*nodePositions[i]; //correct marker positions
      nodePositions[i] = translate+nodePositions[i]; //correct marker positions 
    }
  }

  //############################################################################################################################################
  

  // Create the build spec that uses tags to turn the structure into a real model
  tgBuildSpec spec;
  spec.addBuilder("rod", new tgRodInfo(rodConfig));
  spec.addBuilder("payload", new tgRodInfo(payloadConfig)); 
  spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
  spec.addBuilder("payload_muscle", new tgBasicActuatorInfo(muscleConfig));
    
  // Create your structureInfo
  tgStructureInfo structureInfo(s, spec);

  // Use the structureInfo to build ourselves
  structureInfo.buildInto(*this, world);

  // We could now use tgCast::filter or similar to pull out the
  // models (e.g. muscles) that we want to control. 
  allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
 
  addMarkers(s);

  // call the onSetup methods of all observed things e.g. controllers
  notifySetup();

  // Actually setup the children
  tgModel::setup(world);

  allRods = tgCast::filter<tgModel, tgRod> (getDescendants());
    
  allBaseRigids = tgCast::filter<tgModel, tgBaseRigid> (getDescendants());

  allAbstractMarkers=tgCast::filter<tgModel, abstractMarker> (getDescendants());
  /*
  //figure out bottom 3 nodes
  std::vector<abstractMarker> markers = this -> getMarkers();
  abstractMarker m1=markers[0];
  abstractMarker m2=markers[1];

  double minM=std::min(m1.getWorldPosition().getY(),m2.getWorldPosition().getY());
  if (m1.getWorldPosition().getY()==minM){
  mGrnd1=0;
  }
  else {
  mGrnd1=1;
  }
  for (std::size_t i=2; i<markers.size(); i++){
  abstractMarker mTemp=markers[i];
  double  minM1=std::min(markers[mGrnd1].getWorldPosition().getY(),mTemp.getWorldPosition().getY());
  if (mTemp.getWorldPosition().getY()==minM1){
  mGrnd1=i;
  }
  }
  int start = 0;
  if(start==mGrnd1){
  start++;
  }
  mGrnd2 = start;
  for (std::size_t i=0; i<markers.size(); i++){
  abstractMarker mTemp=markers[i];
  double minM2=std::min(markers[mGrnd2].getWorldPosition().getY(),mTemp.getWorldPosition().getY());
  if (mTemp.getWorldPosition().getY()==minM2 && i!=mGrnd1){
  mGrnd2=i;
  }
  }
  start = 0;
  if(start==mGrnd1 || start==mGrnd2){
  start++;
  }
  mGrnd3 = start;
  for (std::size_t i=0; i<markers.size(); i++){
  abstractMarker mTemp=markers[i];
  double minM3=std::min(markers[mGrnd3].getWorldPosition().getY(),mTemp.getWorldPosition().getY());
  if (mTemp.getWorldPosition().getY()==minM3 && i!=mGrnd1 && i!=mGrnd2){
  mGrnd3 = i;
  }
  }
  */

  
  findBottomNodes();
  /*
  std::cout << std::endl;  
  std::cout << std::endl;
  std::cout << "mgrnd1:" <<  mGrnd[0] << " ";
  std::cout << "mgrnd2:" <<  mGrnd[1] << " ";
  std::cout << "mgrnd3:" <<  mGrnd[2] << std::endl;
  std::cout << markers[mGrnd[0]].getWorldPosition().getY() << " ";
  std::cout << markers[mGrnd[1]].getWorldPosition().getY() << " ";
  std::cout << markers[mGrnd[2]].getWorldPosition().getY() << std::endl;
  */
  
  globalTime = 0;
  toggle = 0;
  tipFlag = 0;

  //std::ofstream sim_outS;
  sim_out.open("simulation_results_just_pretension.txt",std::ios::app);
  sim_out << std::endl;
  
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
  std::vector<tgRod *> payload = find<tgRod>("payload");
  std::vector<abstractMarker> markers = this -> getMarkers();
  const btRigidBody* bt = payload[0]->getPRigidBody();
  btTransform rotation;
  btQuaternion orientation = bt->getOrientation();
  rotation.setRotation(orientation);
  btVector3 unit = btVector3(1,0,0);
  unit = rotation*unit;

  globalTime = globalTime + dt;
  /*
  if(toggle == 0){

    //std::cout << unit << "\n";
    //std::cout<<std::endl;
    
    //for(int i=0;i<markers.size();i++){
    //  std::cout<<markers[i].getWorldPosition().getY()<<" ";
    //}
    
    std::cout<<std::endl;
    std::cout<<markers[mGrnd[0]].getWorldPosition().getY()<<" ";
    std::cout<<markers[mGrnd[1]].getWorldPosition().getY()<<" ";
    std::cout<<markers[mGrnd[2]].getWorldPosition().getY()<<std::endl;
    
    sim_out << unit << "\n";
    sim_out <<std::endl;
    sim_out <<markers[mGrnd[0]].getWorldPosition().getY()<<" ";
    sim_out <<markers[mGrnd[1]].getWorldPosition().getY()<<" ";
    sim_out <<markers[mGrnd[2]].getWorldPosition().getY()<<std::endl;
    toggle = 1;
    sim_out.close();
   
  
  }
  */
  if(globalTime > 1){
    secCount += dt;
    /*
      if(toggle == 1 && globalTime > 12){  
      int check;
      std::cout << "Enter Thrust Value:  ";
      std::cin >> check;
      std::cout << "received " << check << " N as thrust value" << std::endl;
      toggle = 2;
      }	
    */
    if(secCount > 1){
      std::cout << "Sim Time: " << globalTime << std::endl;
      int mGrnd1_orig = mGrnd[0];
      int mGrnd2_orig = mGrnd[1];
      int mGrnd3_orig = mGrnd[2];
      findBottomNodes();
      std::cout << "mgrnd1:" <<  mGrnd[0] << " ";
      std::cout << "mgrnd2:" <<  mGrnd[1] << " ";
      std::cout << "mgrnd3:" <<  mGrnd[2] << std::endl;
      std::cout << "mgrnd1_Y:" <<  markers[mGrnd[0]].getWorldPosition().getY() << " ";
      std::cout << "mgrnd2_Y:" <<  markers[mGrnd[1]].getWorldPosition().getY() << " ";
      std::cout << "mgrnd3_Y:" <<  markers[mGrnd[2]].getWorldPosition().getY() << std::endl;
      btVector3 pos = bt->getCenterOfMassPosition();
      std::cout << "Payload Position,, ";
      std::cout << pos[0] << ", ";
      std::cout << pos[1] << ", ";
      std::cout << pos[2];
      std::cout<<std::endl;
      btVector3 lin_vel = bt->getLinearVelocity();
      std::cout << "Thruster Linear Velocity, ";
      std::cout << lin_vel[0] << ", ";
      std::cout << lin_vel[1] << ", ";
      std::cout << lin_vel[2];
      std::cout<<std::endl;
      btVector3 ang_vel = bt->getAngularVelocity();
      std::cout << "Thruster Angular Velocity, ";
      std::cout << ang_vel[0] << ", ";
      std::cout << ang_vel[1] << ", ";
      std::cout << ang_vel[2];
      std::cout<<std::endl;
      
      std::cout<<std::endl;
      for(int i=0; i<3; i++){
	if(mGrnd[i]!=mGrnd1_orig && mGrnd[i]!=mGrnd2_orig && mGrnd[i]!=mGrnd3_orig){
	  std::cout << "TIPPED!" << std::endl << std::endl ;
	  tipFlag = 1;
	}
      }
      secCount = 0;
    }
  }

  
  if(globalTime > 12 && toggle == 0){
    
    //std::cout << unit << "\n";
    //std::cout<<std::endl;
    
    //for(int i=0;i<markers.size();i++){
    //  std::cout<<markers[i].getWorldPosition().getY()<<" ";
    //}
    
    sim_out << "Tip Flag, ";
    sim_out << tipFlag <<", ";
    sim_out << "Thruster Orientation, ";
    sim_out << unit[0] << ", ";
    sim_out << unit[1] << ", ";
    sim_out << unit[2] << ", ";

    btVector3 pos = bt->getCenterOfMassPosition();
    sim_out << "Thruster Position, ";
    sim_out << pos[0] << ", ";
    sim_out << pos[1] << ", ";
    sim_out << pos[2] << ", ";
    
    btVector3 lin_vel = bt->getLinearVelocity();
    sim_out << "Thruster Linear Velocity, ";
    sim_out << lin_vel[0] << ", ";
    sim_out << lin_vel[1] << ", ";
    sim_out << lin_vel[2] << ", ";

    btVector3 ang_vel = bt->getAngularVelocity();
    sim_out << "Thruster Angular Velocity, ";
    sim_out << ang_vel[0] << ", ";
    sim_out << ang_vel[1] << ", ";
    sim_out << ang_vel[2] << ", ";
    
    sim_out << "Cable Restlengths, ";
    const std::vector<tgBasicActuator*> actuators = this -> getAllActuators();
    for(int i=0; i<allActuators.size(); i++){
      sim_out << actuators[i]->getRestLength() << ", ";
    }
    sim_out << "Cable Tensions, ";
    for(int i=0; i<allActuators.size(); i++){
      sim_out << actuators[i]->getTension() << ", ";
    }
    //sim_out <<std::endl;
    toggle = 1;
    //sim_out.close();
  }
  
  if(globalTime > 25 && toggle == 1){
    
    //std::cout << unit << "\n";
    //std::cout<<std::endl;
    
    //for(int i=0;i<markers.size();i++){
    //  std::cout<<markers[i].getWorldPosition().getY()<<" ";
    //}
    
    btVector3 pos = bt->getCenterOfMassPosition();
    sim_out << "Payload End Position, ";
    sim_out << pos[0] << ", ";
    sim_out << pos[1] << ", ";
    sim_out << pos[2] << ", ";
    
    btVector3 lin_vel = bt->getLinearVelocity();
    sim_out << "Payload End Linear Velocity, ";
    sim_out << lin_vel[0] << ", ";
    sim_out << lin_vel[1] << ", ";
    sim_out << lin_vel[2] << ", ";

    btVector3 ang_vel = bt->getAngularVelocity();
    sim_out << "Payload End Angular Velocity, ";
    sim_out << ang_vel[0] << ", ";
    sim_out << ang_vel[1] << ", ";
    sim_out << ang_vel[2] << ", ";
    sim_out <<std::endl;
    toggle = 2;
    sim_out.close();
  }
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
  markers.clear();
  allAbstractMarkers.clear();
  allActuators.clear();
  allNodes.clear();
  allRods.clear();
  allBaseRigids.clear();
  nodePositions.clear();
  notifyTeardown();
  tgModel::teardown();
}
