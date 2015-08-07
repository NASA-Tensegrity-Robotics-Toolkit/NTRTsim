/*
* Copyright © 2015, United States Government, as represented by the
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
* @file SaddleArmModel.cpp
* @brief Contains the implementation of class SaddleArmModel. Everything  is massless for design purposes. Will implement mass soon.
* Measurements are scaled to average adult male
* $Id$
*/
// This module
#include "SaddleArmModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "core/abstractMarker.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
namespace
{
// see tgBaseString.h for a descripton of some of these rod parameters
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
double pretension_olecranon;
double pretension_anconeus;
double pretension_brachioradialis;
double pretension_support;
double pretension_tricep;
double pretension_bicep; 
bool history;
double maxTens;
double targetVelocity;
} c =
{ 
0.05, // density (kg / length^3)
0.8, // radius (length)
3000.0, // stiffness (kg / sec^2)
200.0, // damping (kg / sec)
15.0, // rod_length (length)
7.5, // rod_space (length)
1.0, // friction (unitless)
0.01, // rollFriction (unitless)
0.2, // restitution (?)
3000.0/1, // pretension_olecranon (force), stiffness/initial length
3000.0/15.55, // pretension_anconeus (force), stiffness/initial length
3000.0/262, // pretension_brachioradialis (force), stiffness/initial length
30000.0/1, // pretension_support (force), stiffness/initial length
30000.0/3.658, // pretension_tricep (force), stiffness/initial length
30000.0/3.81, // pretension_bicep (force), stiffness/initial length
false, // history (boolean)
100000, // maxTens
10000 // targetVelocity
#if (0)
20000 // maxAcc
#endif
// Use the below values for earlier versions of simulation.
// 1.006,
// 0.31,
// 300000.0,
// 3000.0,
// 15.0,
// 7.5,
};
} // namespace
SaddleArmModel::SaddleArmModel() : tgModel()
{
}
SaddleArmModel::~SaddleArmModel()
{
}
void SaddleArmModel::addNodes(tgStructure& s)
{
const double scale = 0.5;
const double bone_scale = 0.3;
const size_t nNodes = 15 + 4 + 8; //4 for massless rod 3 for shoulder
// Average Adult Male Measurements with scale
// Lengths are in mm
const double a = 22 * scale; //ulnta distal width
const double b = 334 * scale * bone_scale; //ulna length
const double c = 265 * scale * bone_scale; //humerus length //NB: in model, c==b
//const double d = 66 * scale; // humerus epicondylar width
//const double e = 246 * scale * bone_scale; //radius length
const double f = 25 * scale; // humerus head radius
const double g = 17 * scale; //ulna proximal width
const double x = a/2;
const double z = c/2;
//Format: (x, z, y)
nodePositions.push_back(btVector3(g, 0, 0)); // 0
nodePositions.push_back(btVector3(0, -g, 0)); //1 
nodePositions.push_back(btVector3(-a/2, 0, 0)); // 2
nodePositions.push_back(btVector3(0, 0, g)); // 3
nodePositions.push_back(btVector3(0, 0, -g)); // 4
nodePositions.push_back(btVector3(0, g, 0)); //5 
nodePositions.push_back(btVector3(0, c, 0)); // 6
nodePositions.push_back(btVector3(x, z, 0)); // 7
nodePositions.push_back(btVector3(b+a/2, -g, 0)); // 8
nodePositions.push_back(btVector3(0, c+2, f)); // 9 
nodePositions.push_back(btVector3(0, c+2, -f)); //10 
//Added 6/17/15
nodePositions.push_back(btVector3(a/2, -2*g, 0)); //11
//ulna and radius
nodePositions.push_back(btVector3(3*a/2, -g, 0)); //12
nodePositions.push_back(btVector3(3*a/4, -g, g)); //13
nodePositions.push_back(btVector3(3*a/4, -g, -g)); //14
nodePositions.push_back(btVector3(f, c+2, 0)); // 15
nodePositions.push_back(btVector3(-f, c+2, 0)); // 16


//Added 7/30/15

//Additional Humerus Nodes to help create saddle 
nodePositions.push_back(btVector3(0, c+2*a/3, g)); // 17
nodePositions.push_back(btVector3(0, c+2*a/3, -g)); // 18

//Shoulder nodes (scapula)
nodePositions.push_back(btVector3(g, c, g)); // 19
nodePositions.push_back(btVector3(g, c, -g)); // 20
nodePositions.push_back(btVector3(-g, c, 0)); // 21
nodePositions.push_back(btVector3(-g-a*3/2, c, 0)); // 22
nodePositions.push_back(btVector3(-g-a*3/2, c-2*g, 0)); // 23

//Shoulder nodes (clavicle)
nodePositions.push_back(btVector3(-c, c+a, 0)); // 24
nodePositions.push_back(btVector3(2*g, c+a, 0)); // 25
nodePositions.push_back(btVector3(0, c+a, 0)); // 26


for(size_t i=0;i<nNodes;i++) {
s.addNode(nodePositions[i][0],nodePositions[i][1],nodePositions[i][2]);
}
}
void SaddleArmModel::addRods(tgStructure& s)
{
// ulna and radius
s.addPair(8, 12, "rod");
s.addPair(12, 14, "rod");
s.addPair(12, 13, "rod");
//olecranon

// triangle olecranon model
//s.addPair(2, 11, "rod");
//s.addPair(0, 11, "rod");
//s.addPair(2, 0, "rod");
//s.addPair(1, 2, "rod");

// Steve's olecranon model
s.addPair(0, 1, "rod");
s.addPair(1, 2, "rod");
s.addPair(1, 11, "rod");

// humerus
s.addPair(3, 5, "rod");
s.addPair(4, 5, "rod");
s.addPair(5, 6, "rod");
s.addPair(6, 17, "rod");
s.addPair(6, 18, "rod");

//shoulder
s.addPair(19, 21, "humerus massless");
s.addPair(20, 21, "humerus massless");
s.addPair(22, 21, "humerus massless");
s.addPair(22, 23, "humerus massless");
s.addPair(21, 23, "humerus massless");

//clavicle
s.addPair(24, 26, "humerus massless");
s.addPair(25, 26, "humerus massless");
}
void SaddleArmModel::addMuscles(tgStructure& s)
{
const std::vector<tgStructure*> children = s.getChildren();
s.addPair(0, 3, "olecranon muscle"); //NB actually fascial tissue
s.addPair(0, 4, "olecranon muscle"); //NB actually fascial tissue
s.addPair(1, 3, "olecranon muscle"); //NB actually fascial tissue
s.addPair(1, 4, "olecranon muscle"); //NB actually fascial tissue
s.addPair(2, 3, "olecranon muscle"); //NB actually fascial tissue
s.addPair(2, 4, "olecranon muscle"); //NB actually fascial tissue
s.addPair(0, 13, "olecranon muscle"); //NB actually fascial tissue
s.addPair(1, 13, "olecranon muscle"); //NB actually fascial tissue
s.addPair(11, 13, "olecranon muscle"); //NB actually fascial tissue
s.addPair(0, 14, "olecranon muscle"); //NB actually fascial tissue
s.addPair(1, 14, "olecranon muscle"); //NB actually fascial tissue
s.addPair(11, 14, "olecranon muscle"); //NB actually fascial tissue
s.addPair(0, 12, "olecranon muscle"); //NB actually fascial tissue
s.addPair(11, 12, "olecranon muscle"); //NB actually fascial tissue
s.addPair(0, 5, "brachioradialis muscle");
s.addPair(2, 5, "olecranon muscle"); //NB actually fascial tissue
s.addPair(3, 13, "right anconeus muscle");
s.addPair(4, 14, "left anconeus muscle");

//added with shoulder 7/30/15

//Anchor muscles (muscles need to change)
s.addPair(26, 17, "olecranon muscle");
s.addPair(26, 18, "olecranon muscle");
s.addPair(25, 6, "olecranon muscle");
s.addPair(26, 6, "olecranon muscle");
s.addPair(24, 6, "olecranon muscle");

//Saddle muscles (muscles need to change)
s.addPair(17, 21, "olecranon muscle");
s.addPair(17, 19, "olecranon muscle");
s.addPair(18, 21, "olecranon muscle"); 
s.addPair(18, 20, "olecranon muscle");

//Tricep Muscles (muscles need to change)
s.addPair(2, 21, "tricep muscle");
s.addPair(2, 6, "tricep muscle");

//Bicep Muscles (muscles need to change)
s.addPair(12, 26, "bicep muscle");
s.addPair(12, 6, "bicep muscle");


}
/*
void SaddleArmModel::addMarkers(tgStructure &s)
{
std::vector<tgRod *> rods=find<tgRod>("rod");
for(int i=0;i<10;i++)
{
const btRigidBody* bt = rods[rodNumbersPerNode[i]]->getPRigidBody();
btTransform inverseTransform = bt->getWorldTransform().inverse();
btVector3 pos = inverseTransform * (nodePositions[i]);
abstractMarker tmp=abstractMarker(bt,pos,btVector3(0.08*i,1.0 - 0.08*i,.0),i);
this->addMarker(tmp);
}
}
*/
void SaddleArmModel::setup(tgWorld& world)
{
const tgRod::Config rodConfig(c.radius, c.density, c.friction, c.rollFriction, c.restitution);
const tgRod::Config rodConfigMassless(c.radius, 0.00/*c.density*/, c.friction, c.rollFriction, c.restitution);

/// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate
tgBasicActuator::Config olecranonMuscleConfig(c.stiffness, c.damping, c.pretension_olecranon, c.history, c.maxTens, c.targetVelocity);
tgBasicActuator::Config anconeusMuscleConfig(c.stiffness, c.damping, c.pretension_anconeus, c.history, c.maxTens, c.targetVelocity);
tgBasicActuator::Config brachioradialisMuscleConfig(c.stiffness, c.damping, c.pretension_brachioradialis, c.history, c.maxTens, c.targetVelocity);
tgBasicActuator::Config supportstringMuscleConfig(c.stiffness, c.damping, c.pretension_support, c.history, c.maxTens, c.targetVelocity);

//added 8/7/15
tgBasicActuator::Config tricepMuscleConfig(c.stiffness, c.damping, c.pretension_tricep, c.history, c.maxTens, c.targetVelocity);
tgBasicActuator::Config bicepMuscleConfig(c.stiffness, c.damping, c.pretension_bicep, c.history, c.maxTens, c.targetVelocity);

// Start creating the structure
tgStructure s;
addNodes(s);
addRods(s);
addMuscles(s);
// Move the arm out of the ground
btVector3 offset(0.0, 50.0, 0.0);
s.move(offset);
// Create the build spec that uses tags to turn the structure into a real model
tgBuildSpec spec;
spec.addBuilder("massless", new tgRodInfo(rodConfigMassless));
spec.addBuilder("rod", new tgRodInfo(rodConfig));
spec.addBuilder("olecranon muscle", new tgBasicActuatorInfo(olecranonMuscleConfig));
spec.addBuilder("anconeus muscle", new tgBasicActuatorInfo(anconeusMuscleConfig));
spec.addBuilder("brachioradialis muscle", new tgBasicActuatorInfo(brachioradialisMuscleConfig));
spec.addBuilder("support muscle", new tgBasicActuatorInfo(supportstringMuscleConfig));

//added 8/7/15
spec.addBuilder("tricep muscle", new tgBasicActuatorInfo(tricepMuscleConfig));
spec.addBuilder("bicep muscle", new tgBasicActuatorInfo(bicepMuscleConfig));

// Create your structureInfo
tgStructureInfo structureInfo(s, spec);
// Use the structureInfo to build ourselves
structureInfo.buildInto(*this, world);
// We could now use tgCast::filter or similar to pull out the
// models (e.g. muscles) that we want to control.
allMuscles = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
// call the onSetup methods of all observed things e.g. controllers
notifySetup();
// Actually setup the children
tgModel::setup(world);
//map the rods and add the markers to them
//addMarkers(s);
}
void SaddleArmModel::step(double dt)
{
// Precondition
if (dt <= 0.0) {
throw std::invalid_argument("dt is not positive");
} else {
// Notify observers (controllers) of the step so that they can take action
notifyStep(dt);
tgModel::step(dt); // Step any children
}
}
void SaddleArmModel::onVisit(tgModelVisitor& r)
{
tgModel::onVisit(r);
}
const std::vector<tgBasicActuator*>& SaddleArmModel::getAllMuscles() const
{
return allMuscles;
}
void SaddleArmModel::teardown()
{
notifyTeardown();
tgModel::teardown();
}
