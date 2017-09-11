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

// This module
#include "SUPERballV2Model.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "core/tgBox.h"
#include "core/tgSphere.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgEmptyInfo.h"
#include "tgcreator/tgSphereInfo.h"
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
     6/(M_PI*(0.05/2.0*10.0)*(0.05/2.0*10.0)*1.65*10.0)/5,    // density (kg / length^3)
     0.05/2.0*10,     // radius (deca-meters)
     1600.0,   // stiffness (kg / sec^2)
     80.0,    // damping (kg / sec)
     1.65*10.0,     // rod_length (length)
     7.5,      // rod_space (length)
     0.9,      // friction (unitless)
     0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)
     0,        // pretension (in deca-N)
     false,			// History logging (boolean)
     100000,   // maxTens
     100,    // targetVelocity

     // Use the below values for earlier versions of simulation.
     // 1.006,    
     // 0.31,     
     // 300000.0, 
     // 3000.0,   
     // 15.0,     
     // 7.5,      
  };
} // namespace

void addBarNodesWithCoMAzDec(tgStructure& s, double comx, double comy, double comz,
                             double azimuth, double declination,
                             double length);
                            
btVector3 azimuthDeclinationToVector(double azimuth, double declination);


SUPERballV2Model::SUPERballV2Model() : tgModel() 
{
}

SUPERballV2Model::~SUPERballV2Model()
{
}


void SUPERballV2Model::moveRodsToEqMani(tgStructure* rod1, tgStructure* rod2, tgStructure* rod3, tgStructure* rod4, tgStructure* rod5, tgStructure* rod6, double alpha, double delta, double b, double l)
{
    double u = sin(delta) * cos(alpha + M_PI/6.0);
    double h = cos(delta)/(2.0*u) * (l*u + sqrt(b*b/3.0 - 3.0*l*l*u*u) - b/sqrt(3.0));
    btVector3 initOrientation = btVector3(1, 0, 0);
    
    // The bar labeling and parametrization come from (Sultan 2001).
    // Bar 11, nodes 0, 1
    btVector3 bar1Orient = azimuthDeclinationToVector(alpha, delta);
    rod1->addRotation(btVector3(0,0,0), initOrientation, bar1Orient);
    rod1->move(btVector3(l/2.0*sin(delta)*cos(alpha)-b/2.0, 
                         l/2.0*sin(delta)*sin(alpha) - b/(2.0*sqrt(3.0)),
                         l/2.0*cos(delta) ) );
                         
    // Bar 21, nodes 2, 3
    btVector3 bar2Orient = azimuthDeclinationToVector(alpha + 4.0*M_PI/3.0, delta);
    rod2->addRotation(btVector3(0,0,0), initOrientation, bar2Orient);
    rod2->move(btVector3(l/2.0*sin(delta)*cos(alpha + 4.0*M_PI/3.0),
                            b/sqrt(3.0) + l/2.0*sin(delta)*sin(alpha + 4.0*M_PI/3.0),
                            l/2.0*cos(delta) ) );
    // Bar 31, nodes 4, 5
    btVector3 bar3Orient = azimuthDeclinationToVector(alpha + 2.0*M_PI/3.0, delta);
    rod3->addRotation(btVector3(0,0,0), initOrientation, bar3Orient);
    rod3->move(btVector3(b/2.0 + l/2.0*sin(delta)*cos(alpha + 2.0*M_PI/3.0),
                            l/2.0*sin(delta)*sin(alpha + 2.0*M_PI/3.0) - b/(2.0*sqrt(3.0)),
                            l/2.0*cos(delta) ) );
                            
    // Bar 12, nodes 6, 7
    btVector3 bar4Orient = azimuthDeclinationToVector(alpha + 2.0*M_PI/3.0, delta);
    rod4->addRotation(btVector3(0,0,0), initOrientation, bar4Orient);
    rod4->move(btVector3(l/4*sin(delta)*cos(alpha) + sqrt(3.0)/4.0*l*sin(delta)*sin(alpha) - b/2.0,
     b/(2.0*sqrt(3.0)) - sqrt(3.0)/4.0*l*sin(delta)*cos(alpha) + l/4.0*sin(delta)*sin(alpha),
                            3.0/2.0*l*cos(delta) - h ) );
    
    // Bar 22, nodes 8, 9
    btVector3 bar5Orient = azimuthDeclinationToVector(alpha, delta);
    rod5->addRotation(btVector3(0,0,0), initOrientation, bar5Orient);
    rod5->move(btVector3(b/2.0 - l/2.0*sin(delta)*cos(alpha),
                            b/(2.0*sqrt(3.0)) - l/2.0*sin(delta)*sin(alpha),
                            3.0/2.0*l*cos(delta) - h ) );
    
    // Bar 32, nodes 10, 11
    btVector3 bar6Orient = azimuthDeclinationToVector(alpha + 4.0*M_PI/3.0, delta);
    rod6->addRotation(btVector3(0,0,0), initOrientation, bar6Orient);
    rod6->move(btVector3(l/4.0*sin(delta)*cos(alpha) - sqrt(3.0)/4.0*l*sin(delta)*sin(alpha),
           l/4.0*sin(delta)*sin(alpha) + sqrt(3.0)/4.0*l*sin(delta)*cos(alpha) - b/sqrt(3.0),
                            3.0/2.0*l*cos(delta) - h ) );
                            
                         /*
    // Bar 21, nodes 2, 3
    addBarNodesWithCoMAzDec(s, l/2.0*sin(delta)*cos(alpha + 4.0*M_PI/3.0),
                            b/sqrt(3.0) + l/2.0*sin(delta)*sin(alpha + 4.0*M_PI/3.0),
                            l/2.0*cos(delta),
                            alpha + 4.0*M_PI/3.0, delta, l);
    // Bar 31, nodes 4, 5
    addBarNodesWithCoMAzDec(s, b/2.0 + l/2.0*sin(delta)*cos(alpha + 2.0*M_PI/3.0),
                            l/2.0*sin(delta)*sin(alpha + 2.0*M_PI/3.0) - b/(2.0*sqrt(3.0)),
                            l/2.0*cos(delta),
                            alpha + 2.0*M_PI/3.0, delta, l);
    // Bar 12, nodes 6, 7
    addBarNodesWithCoMAzDec(s,
                   l/4*sin(delta)*cos(alpha) + sqrt(3.0)/4.0*l*sin(delta)*sin(alpha) - b/2.0,
     b/(2.0*sqrt(3.0)) - sqrt(3.0)/4.0*l*sin(delta)*cos(alpha) + l/4.0*sin(delta)*sin(alpha),
                            3.0/2.0*l*cos(delta) - h,
                            alpha + 2.0*M_PI/3.0, delta, l);
    // Bar 22, nodes 8, 9
    addBarNodesWithCoMAzDec(s, b/2.0 - l/2.0*sin(delta)*cos(alpha),
                            b/(2.0*sqrt(3.0)) - l/2.0*sin(delta)*sin(alpha),
                            3.0/2.0*l*cos(delta) - h,
                            alpha, delta, l);
    // Bar 32, nodes 10, 11
    addBarNodesWithCoMAzDec(s,
                        l/4.0*sin(delta)*cos(alpha) - sqrt(3.0)/4.0*l*sin(delta)*sin(alpha),
           l/4.0*sin(delta)*sin(alpha) + sqrt(3.0)/4.0*l*sin(delta)*cos(alpha) - b/sqrt(3.0),
                            3.0/2.0*l*cos(delta) - h,
                            alpha + 4.0*M_PI/3.0, delta, l);
    std::cout << "]" << std::endl;
    */
}

void SUPERballV2Model::addActuators(tgStructure* rod1, tgStructure* rod2, tgStructure* rod3, tgStructure* rod4, tgStructure* rod5, tgStructure* rod6)
{
    // Get nodes from each rods:
    int n0_idx = 3; // Child index of what to attach to.
    int n1_idx = 4;
    std::vector<tgStructure*> r1children = rod1->getChildren();
    tgNode n0 = r1children[n0_idx]->getNodes()[0];
    tgNode n1 = r1children[n1_idx]->getNodes()[0];
    std::vector<tgStructure*> r2children = rod2->getChildren();
    tgNode n2 = r2children[n0_idx]->getNodes()[0];
    tgNode n3 = r2children[n1_idx]->getNodes()[0];
    std::vector<tgStructure*> r3children = rod3->getChildren();
    tgNode n4 = r3children[n0_idx]->getNodes()[0];
    tgNode n5 = r3children[n1_idx]->getNodes()[0];
    std::vector<tgStructure*> r4children = rod4->getChildren();
    tgNode n6 = r4children[n0_idx]->getNodes()[0];
    tgNode n7 = r4children[n1_idx]->getNodes()[0];
    std::vector<tgStructure*> r5children = rod5->getChildren();
    tgNode n8 = r5children[n0_idx]->getNodes()[0];
    tgNode n9 = r5children[n1_idx]->getNodes()[0];
    std::vector<tgStructure*> r6children = rod6->getChildren();
    tgNode n10 = r6children[n0_idx]->getNodes()[0];
    tgNode n11 = r6children[n1_idx]->getNodes()[0];
    
    // Add the actuators:
    // Saddle
    rod1->addPair(n5, n8,  "muscle"); // 0
    rod1->addPair(n8, n3,  "muscle");
    rod1->addPair(n3, n6,  "muscle");
    rod1->addPair(n6, n1,  "muscle");
    rod1->addPair(n1, n10,  "muscle");
    rod1->addPair(n10, n5,  "muscle");
    
    // Vertical
    rod1->addPair(n0, n5,  "muscle"); // 6
    rod1->addPair(n4, n3,  "muscle");
    rod1->addPair(n2, n1,  "muscle");
    rod1->addPair(n10, n7,  "muscle");
    rod1->addPair(n6, n9,  "muscle");
    rod1->addPair(n8, n11,  "muscle");
    
    // Diagonal
    rod1->addPair(n0, n10,  "muscle"); // 12
    rod1->addPair(n2, n6,  "muscle");
    rod1->addPair(n4, n8,  "muscle");
    rod1->addPair(n1, n7,  "muscle");
    rod1->addPair(n3, n9,  "muscle");
    rod1->addPair(n5, n11,  "muscle");
    
    // Boundary/Base
    rod1->addPair(n0, n4,  "muscle"); // 18
    rod1->addPair(n4, n2,  "muscle");
    rod1->addPair(n2, n0,  "muscle");
    rod1->addPair(n7, n11,  "muscle");
    rod1->addPair(n11, n9,  "muscle");
    rod1->addPair(n9, n7,  "muscle");
}


void SUPERballV2Model::setup(tgWorld& world)
{
    // Rod properties. Specify color and texture usage.
    double rodLength = 1.55*10.0; // deca-meters
    double rodRadius = 0.04/2.0 * 10.0;
    double rodMass = 4.0; // kg
    double rodDensity = rodMass / (M_PI*rodRadius*rodRadius*rodLength);
    tgRod::Config rodConfig(rodRadius, rodDensity, 0.9, 
				0.5, 0.1, btVector3(0.6, 0.6, 0.6), true);
    
    // Actuator/string properties
    tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension, c.hist, 
					    c.maxTens);
	
	// Motor properties.
	double motorXDim = 0.11*10.0; // deca-meters
	double motorYDim = 0.07*10.0;
	double motorZDim = 0.05*10.0;
	double motor_rod_offsetZ = rodRadius + motorZDim/2.0;
	double motor_rod_offsetX = 0.04*10.0;
	double motorMass = 0.25; // kg
	double motorDensity = motorMass / (motorXDim*motorYDim*motorZDim);
	double motorZRot = 0.4;
	double motorFriction = 0.9;
	double motorRollfriction = 0.5;
	double motorRestitution = 0.1;
	btVector3 motorColor = btVector3(0.7,0,0);
	tgBox::Config motorConfig(motorYDim/2.0, motorZDim/2.0, motorDensity, motorFriction, motorRollfriction,
	             motorRestitution, motorColor, true);
	tgBox::Config motorConfigRefl(motorZDim/2.0, motorYDim/2.0, motorDensity, motorFriction, motorRollfriction,
	             motorRestitution, motorColor, true);
	             
    // Cable attachment props
	double CAXDim = 0.05*10.0; // deca-meters
	double CAYDim = 0.12*10.0; // This argument sets how far cables are attached.
	double CAZDim = 0.1*10.0;
	double CAMass = 0.2; // kg
	double CADensity = CAMass / (CAXDim*CAYDim*CAZDim);
	double CA_offsX = CAXDim;
	tgBox::Config CAConfig(CAYDim/2.0, CAZDim/2.0, CADensity, 0.9, 0.01,
	             0, btVector3(0.4,0.4,0.4), false);
	
	// Endcap properties
	double endcapRadius = 0.10/2.0*10;
	double endcapMass = 0.3; // kg
	double endcapDensity = endcapMass / (4.0/3.0*M_PI*endcapRadius*endcapRadius*endcapRadius);
	double endcap_offsX = endcapRadius - 0.02*10.0;
	btVector3 endcapColor = btVector3(0.1,0.1,0.1);
	tgSphere::Config endcapConfig(endcapRadius, endcapDensity, 0.9, 0.9, 
	        0.2, endcapColor, false);
	        
	std::cout << "Rod net mass: " <<
	        (rodMass + 4*motorMass + 2*CAMass + 2*endcapMass) << " kg" << std::endl;
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("bar", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("motor", new tgBoxInfo(motorConfig));
    spec.addBuilder("motorR", new tgBoxInfo(motorConfigRefl));
    spec.addBuilder("cableAttach", new tgBoxInfo(CAConfig));
    spec.addBuilder("endcap", new tgSphereInfo(endcapConfig));
    spec.addBuilder("space", new tgEmptyInfo());
    
    // Create a single SUPERball rod:
    // Create metal cylinder:
    tgStructure rod;
    rod.addNode(0, 0, 0); // 0
    rod.addNode(rodLength, 0, 0); // 1
    rod.addPair(0, 1, "bar");
    
    // Attachment points to rod:
    rod.addNode(motor_rod_offsetX, 0, -motor_rod_offsetZ); // 2
    rod.addNode(motor_rod_offsetX, 0, motor_rod_offsetZ); // 3
    rod.addNode(-CA_offsX, 0, 0); // 4
    rod.addNode(-CA_offsX-endcap_offsX, 0, 0); // 5
    rod.addNode(rodLength - motorXDim -0.07*10.0, 0, 0); // 6
    rod.addNode(rodLength +CA_offsX, 0, 0); // 7
    rod.addNode(rodLength+CA_offsX+endcap_offsX, 0, 0); // 8
    rod.addPair(0, 2, "space");
    rod.addPair(0, 3, "space");
    rod.addPair(0, 4, "space");
    rod.addPair(0, 5, "space");
    rod.addPair(0, 6, "space");
    rod.addPair(0, 7, "space");
    rod.addPair(0, 8, "space");
    
    // Bottom motors
    tgStructure motor_bot1;
    motor_bot1.addNode(0, 0, 0); // 0
    motor_bot1.addNode(motorXDim, 0, 0); //1
    motor_bot1.addPair(0, 1, "motor");
    motor_bot1.addRotation(btVector3(0, 0, 0), btVector3(0, 0, 1), motorZRot);
    motor_bot1.move(btVector3(motor_rod_offsetX, 0, -motor_rod_offsetZ));
    rod.addChild(motor_bot1);
    
    tgStructure motor_bot2;
    motor_bot2.addNode(0, 0, 0); // 0
    motor_bot2.addNode(motorXDim, 0, 0); //1
    motor_bot2.addPair(0, 1, "motor");
    motor_bot2.addRotation(btVector3(0, 0, 0), btVector3(0, 0, 1), -motorZRot);
    motor_bot2.move(btVector3(motor_rod_offsetX, 0, motor_rod_offsetZ));
    rod.addChild(motor_bot2);
    
    // Top motors
    tgStructure topMotors;
    topMotors.addNode(motor_rod_offsetX, 0, -motor_rod_offsetZ); // 1
    topMotors.addNode(motor_rod_offsetX, 0, motor_rod_offsetZ); // 2
    topMotors.addNode(0, 0, 0); // 0
    topMotors.addPair(0, 1, "space");
    topMotors.addPair(0, 2, "space");
    
    tgStructure motor_top1;
    tgStructure motor_top2;
    motor_top1.addNode(0, 0, 0); // 0
    motor_top1.addNode(motorXDim, 0, 0); //1
    motor_top1.addPair(0, 1, "motorR");
    motor_top1.addRotation(btVector3(0, 0, 0), btVector3(0, 0, 1), motorZRot);
    motor_top1.move(btVector3(motor_rod_offsetX, 0, -motor_rod_offsetZ));
    topMotors.addChild(motor_top1);
    motor_top2.addNode(0, 0, 0); // 0
    motor_top2.addNode(motorXDim, 0, 0); //1
    motor_top2.addPair(0, 1, "motorR");
    motor_top2.addRotation(btVector3(0, 0, 0), btVector3(0, 0, 1), -motorZRot);
    motor_top2.move(btVector3(motor_rod_offsetX, 0, motor_rod_offsetZ));
    topMotors.addChild(motor_top2);
    
    topMotors.addRotation(btVector3(0, 0, 0), btVector3(1, 0, 0), M_PI/2.0);
    topMotors.move(btVector3(rodLength - motorXDim - 0.07*10.0, 0, 0));
    rod.addChild(topMotors);
    
    // Add cable attachment holders:
    tgStructure CA_bot;
    CA_bot.addNode(0, 0, 0); // 0
    CA_bot.addNode(CAXDim, 0, 0); //1
    CA_bot.addPair(0, 1, "cableAttach");
    CA_bot.move(btVector3(-CA_offsX, 0, 0));
    rod.addChild(CA_bot);
    
    tgStructure CA_top;
    CA_top.addNode(0, 0, 0); // 0
    CA_top.addNode(-CAXDim, 0, 0); //1
    CA_top.addPair(0, 1, "cableAttach");
    CA_top.move(btVector3(rodLength+CA_offsX, 0, 0));
    CA_top.addRotation(btVector3(0, 0, 0), btVector3(1, 0, 0), M_PI/2.0);
    rod.addChild(CA_top);
    
    // Add endcaps:
    tgStructure bot_endcap;
    bot_endcap.addNode(0, 0, 0, "endcap");
    bot_endcap.move(btVector3(-CA_offsX-endcap_offsX, 0, 0));
    rod.addChild(bot_endcap);
    tgStructure top_endcap;
    top_endcap.addNode(0, 0, 0, "endcap");
    top_endcap.move(btVector3(rodLength+CA_offsX+endcap_offsX, 0, 0));
    rod.addChild(top_endcap);
    
    rod.move(btVector3(-rodLength/2.0, 0, 0)); // Move so COM is at 0.
    // Note, principle axis is X.
    
    // Now, we have created a single rod. Let's create the entire SUPERball:
    tgStructure* rod2 = new tgStructure(rod);
    tgStructure* rod3 = new tgStructure(rod);
    tgStructure* rod4 = new tgStructure(rod);
    tgStructure* rod5 = new tgStructure(rod);
    tgStructure* rod6 = new tgStructure(rod);
    
    // Move rods to desired initial position:
    double alpha_i = 60.0/180.0*M_PI;
    double delta_i = 55.0/180.0*M_PI;
    double l = 1.65*10.0;
    double b = sqrt(3.0/8.0)*l;
    moveRodsToEqMani(&rod, rod2, rod3, rod4, rod5, rod6, alpha_i, delta_i, b, l);
    
    // Add actuators:
    addActuators(&rod, rod2, rod3, rod4, rod5, rod6);
    
    
    // Assemble SUPERball
    tgStructure SUPERball;
    SUPERball.addChild(rod);
    SUPERball.addChild(rod2);
    SUPERball.addChild(rod3);
    SUPERball.addChild(rod4);
    SUPERball.addChild(rod5);
    SUPERball.addChild(rod6);
    
    // Rotate from Sultan x,y,z(up) -> Bullet x,y(up),z:
    btVector3 rotationPoint = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis = btVector3(1, 0, 0); // x-axis rotation
    double rotationAngle = M_PI/2;
    SUPERball.addRotation(rotationPoint, rotationAxis, rotationAngle);
    rotationPoint = btVector3(0, 0, 0); // origin
    rotationAxis = btVector3(0, 0, 1); // z-axis rotation
    rotationAngle = M_PI;
    SUPERball.addRotation(rotationPoint, rotationAxis, rotationAngle);
    
    // Rotate so face (10,5,0)+0 is on ground:
    SUPERball.addRotation(btVector3(0,0,0), btVector3(0,0,1), -1.2248);
    
    // Move to above ground level (y=0)
    SUPERball.move(btVector3(0, 0.8*10.0, 0));
    
    // Create a structureInfo structure to represent our model
    tgStructureInfo structureInfo(SUPERball, spec);

    // Use the structureInfo to build models into the Bullet world
    structureInfo.buildInto(*this, world);

    // Pull out the models that we want to control. 
    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
    
    
    /*
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
    
    s.move(btVector3(0, 2, 0)); // Move CoM to a certain point.
    
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
    */
}

void SUPERballV2Model::step(double dt)
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

void addBarNodesWithCoMAzDec(tgStructure& s, double comx, double comy, double comz,
                             double azimuth, double declination,
                             double length)
{
    // Node 1:
    double nx = comx - length/2.0*sin(declination)*cos(azimuth);
    double ny = comy - length/2.0*sin(declination)*sin(azimuth);
    double nz = comz - length/2.0*cos(declination);
    std::cout << nx << "," << ny << "," << nz << std::endl;
    s.addNode(nx, ny, nz);
    
    // Node 2:
    nx = comx + length/2.0*sin(declination)*cos(azimuth);
    ny = comy + length/2.0*sin(declination)*sin(azimuth);
    nz = comz + length/2.0*cos(declination);
    std::cout << nx << "," << ny << "," << nz << std::endl;
    s.addNode(nx, ny, nz);
}

btVector3 azimuthDeclinationToVector(double azimuth, double declination) {
    return btVector3(
    sin(declination)*cos(azimuth),
    sin(declination)*sin(azimuth),
    cos(declination) );
}

void SUPERballV2Model::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& SUPERballV2Model::getAllActuators() const
{
    return allActuators;
}
    
void SUPERballV2Model::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
