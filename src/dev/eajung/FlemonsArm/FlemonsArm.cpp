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
 * @file FlemonsArm.cpp
 * @brief Contains the definition of the members of the class FlemonsArm.
 * $Id$
 */

// This module
#include "FlemonsArm.h"
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

/**
 * Anonomous namespace so we don't have to declare the config in
 * the header.
 */
namespace
{
    /**
     * Configuration parameters so they're easily accessable.
     * All parameters must be positive.
     */
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double damping;
        double pretension_olecranon;
	double pretension_anconeus;
	double pretension_brachioradialis;
        double humerus; 
	double ulna; 
	double mass_fixed;

	//additional parameters set by steve, ask him to explain why
	double rollFriction;
	double friction;
	double restitution;
	double pretension_support;
	double maxTens;
	double targetVelocity;
	bool history;
    } c =
   {
       0.05,     // density (mass / length^3)
       0.8,     // radius (length)
       3000.0,   // stiffness (mass / sec^2)
       200.0,     // damping (mass / sec)
       3000.0,     // pretension of olecranon (mass * length / sec^2)
       3000.0/15.55, // pretension of anconeus (mass * length / sec^2)
       3000.0/262, // pretension of brachioradialis (mass * length / sec^2)	
       15.0,     // humerus (length)
       7.5, 	//ulna (length)
       0, // mass is fixed

       0.01, // rollFriction (unitless)
       1.0, //friction
       0.2, // restitution
       30000.0/1, //pretension_support (force), stiffness/initial length
       100000, //maxTens (force)
       10000, //targetVelocity
       false //history (boolean)
  };
} // namespace

FlemonsArmModel::FlemonsArmModel() :
tgModel() 
{
}

FlemonsArmModel::~FlemonsArmModel()
{
}

void FlemonsArmModel::addNodes(tgStructure& s)
{
	//bottom right (index: 0)
	//s.addNode(-edge, 0, 0);

	const double scale = 0.5;
	const double bone_scale = 0.3;
	const size_t nNodes = 31; //number of nodes

	//all in mm for measurements
	//const double humerusNode = 36.58; // scale * bone_scale * 243.86
	//const double ulnaNode = 28.2; // scale * bone_scale * 188

	//Node Positions: Bones

	//nodePositions.push_back(btVector3(0, humerusNode, 0)); // index: 0
	//nodePositions.push_back(btVector3(0, 0, 0)); // index: 1
	
	//nodePositions.push_back(btVector3(-ulnaNode, 0.0001, 0)); // index: 2
	
	//nodePositions.push_back(btVector3(0, 0.0001, 0)); //index: 3
	
	//Node Positions: Muscles

	//nodePositions.push_back(btVector3(-ulnaNode/2, 0.0001, 0)); // index: 4 halfway in the ulna

	//nodePositions.push_back(btVector3(0, humerusNode/2, 0)); // index: 5


// Average Adult Male Measurements with scale
// Lengths are in mm
const double ulna_w = 22 * scale; //ulnta distal width
const double ulna_l = 334 * scale * bone_scale; //ulna length
const double humerus_l = 265 * scale * bone_scale; //humerus length //NB: in model, c==b
//const double d = 66 * scale; // humerus epicondylar width
//const double e = 246 * scale * bone_scale; //radius length
const double humerus_head_r = 25 * scale; // humerus head radius
const double g = 17 * scale; //ulna proximal width
const double x = ulna_w/2;
const double z = humerus_l/2;
//Format: (x, z, y)
nodePositions.push_back(btVector3(g, 0, 0)); // 0
nodePositions.push_back(btVector3(0, -g, 0)); //1 
nodePositions.push_back(btVector3(-ulna_w/2, 0, 0)); // 2
nodePositions.push_back(btVector3(0, 0, g)); // 3
nodePositions.push_back(btVector3(0, 0, -g)); // 4
nodePositions.push_back(btVector3(0, g, 0)); //5 
nodePositions.push_back(btVector3(0, humerus_l, 0)); // 6
nodePositions.push_back(btVector3(x, z, 0)); // 7
nodePositions.push_back(btVector3(ulna_l+ulna_w/2, -g, 0)); // 8
nodePositions.push_back(btVector3(0, humerus_l+2, humerus_head_r)); // 9 
nodePositions.push_back(btVector3(0, humerus_l+2, -humerus_head_r)); //10 
//Added 6/17/15
nodePositions.push_back(btVector3(ulna_w/2, -2*g, 0)); //11
//ulna and radius
nodePositions.push_back(btVector3(3*ulna_w/2, -g, 0)); //12
nodePositions.push_back(btVector3(3*ulna_w/4, -g, g)); //13
nodePositions.push_back(btVector3(3*ulna_w/4, -g, -g)); //14
nodePositions.push_back(btVector3(humerus_head_r, humerus_l+2, 0)); // 15
nodePositions.push_back(btVector3(-humerus_head_r, humerus_l+2, 0)); // 16

//Added 7/13/15

//upper humerus nodes
nodePositions.push_back(btVector3(g/2,humerus_l-2,x/2)); //17
nodePositions.push_back(btVector3(-g/2,humerus_l-2,x/2)); //18
nodePositions.push_back(btVector3(-g/2,humerus_l-2,-x/2)); //19
nodePositions.push_back(btVector3(g/2,humerus_l-2,-x/2)); //20

//lower humerus nodes
nodePositions.push_back(btVector3(x/3,g+7.5,g/2)); //21
nodePositions.push_back(btVector3(x/3,g+7.5,-g/2)); //22
nodePositions.push_back(btVector3(x/3,g+5,g/2)); //23
nodePositions.push_back(btVector3(x/3,g+5,-g/2)); //24

//ulna/radius nodes
nodePositions.push_back(btVector3(ulna_l/2 -1,g+6,-x/4)); //25
nodePositions.push_back(btVector3(-ulna_l/12,g+6,-x/4)); //26
nodePositions.push_back(btVector3(ulna_l/2 -1,g+6,x/4)); //27
nodePositions.push_back(btVector3(-ulna_l/12,g+6,x/4)); //28

//elbow nodes
nodePositions.push_back(btVector3(ulna_l/9,g+10,0)); //29
nodePositions.push_back(btVector3(ulna_l/9,g+2,0)); //30

//nodePositions.push_back(btVector3(0,humerus_l+0.001,0)); //31
//nodePositions.push_back(btVector3(0,humerus_l+0.002, 0)); //32

	for(size_t i = 0; i < nNodes; i++){
		s.addNode(nodePositions[i][0], nodePositions[i][1], nodePositions[i][2]);
	}
}

void FlemonsArmModel::addRods(tgStructure& s)
{
	//vertical sides
	//s.addPair( 0, 4, "rod");

	//s.addPair(0, 5, "humerus massless");
	//s.addPair(1, 5, "humerus massless");
	
	//s.addPair(3, 4, "rod");
	//s.addPair(4, 2, "rod");
/*
// ulna and radius
s.addPair(8, 12, "rod");
s.addPair(12, 14, "rod");
s.addPair(12, 13, "rod");
//olecranon
s.addPair(2, 11, "rod");
s.addPair(0, 11, "rod");
s.addPair(2, 0, "rod");
s.addPair(1, 2, "rod");
// humerus
s.addPair(3, 5, "humerus massless");
s.addPair(4, 5, "humerus massless");
s.addPair(5, 6, "humerus massless");
*/

	//humerus
	s.addPair(5, 6, "humerus massless"); //humerus bone
	s.addPair(17, 18, "rod"); //upper humerus parallel 1
	s.addPair(19, 20, "rod"); //upper humerus parallel 2
	s.addPair(21, 22, "rod"); //lower humerus parallel 1
	s.addPair(23, 24, "rod"); //lower humerus parallel 2

	s.addPair(25, 26, "rod"); //ulna 1
	s.addPair(27, 28, "rod"); //ulna 2

	s.addPair(29, 30, "rod"); //elbow 
}

void FlemonsArmModel::addMuscles(tgStructure& s)
{

	s.addPair(6, 17, "anconeus muscle");
	s.addPair(6, 18, "anconeus muscle");
	s.addPair(6, 19, "anconeus muscle");
	s.addPair(6, 20, "anconeus muscle");
	s.addPair(19, 18, "anconeus muscle");
	s.addPair(20, 17, "anconeus muscle");

	s.addPair(17, 21, "anconeus muscle");
	s.addPair(20, 22, "anconeus muscle");
	s.addPair(21, 23, "anconeus muscle");
	s.addPair(22, 24, "anconeus muscle");

	s.addPair(22, 26, "olecranon muscle");
	s.addPair(24, 26, "olecranon muscle");
	s.addPair(19, 26, "olecranon muscle");

	s.addPair(21, 28, "olecranon muscle");
	s.addPair(23, 28, "olecranon muscle");
	s.addPair(18, 28, "olecranon muscle");

	s.addPair(25, 27, "anconeus muscle");
	s.addPair(26, 28, "olecranon muscle");
	s.addPair(5, 23, "olecranon muscle");
	s.addPair(5, 24, "olecranon muscle");
	s.addPair(5, 26, "olecranon muscle");
	s.addPair(5, 28, "olecranon muscle");
	
	//elbow
	s.addPair(29, 21, "olecranon muscle");
	s.addPair(29, 22, "olecranon muscle");
	s.addPair(30, 23, "olecranon muscle");
	s.addPair(30, 24, "olecranon muscle");



/*
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
*/	
}

void FlemonsArmModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config rodConfig(c.radius, c.density, c.friction, c.rollFriction, c.restitution);
    const tgRod::Config rodConfigMassless(c.radius, 0.0/*c.density*/, c.friction, c.rollFriction, c.restitution);
    
    //have Steve explain 
    //Olecranon Muscle Configuration
    tgBasicActuator::Config olecranonMuscleConfig(c.stiffness, c.damping, c.pretension_olecranon, c.history, c.maxTens, c.targetVelocity);
    //Anconeus Muscle Configuration
    tgBasicActuator::Config anconeusMuscleConfig(c.stiffness, c.damping, c.pretension_anconeus, c.history, c.maxTens, c.targetVelocity);
    //Brachioradialis Muscle Configuration
    tgBasicActuator::Config brachioradialisMuscleConfig(c.stiffness, c.damping, c.pretension_brachioradialis, c.history, c.maxTens, c.targetVelocity);
    //Support String Muscle Configuration
    tgBasicActuator::Config supportstringMuscleConfig(c.stiffness, c.damping, c.pretension_support, c.history, c.maxTens, c.targetVelocity);

    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the structure
    addNodes(s);
    
    // Add rods to the structure
    addRods(s);
    
    // Add muscles to the structure
    addMuscles(s);
    
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 50.0, 0));
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;

    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("massless", new tgRodInfo(rodConfigMassless));    
    spec.addBuilder("olecranon muscle", new tgBasicActuatorInfo(olecranonMuscleConfig));
    spec.addBuilder("anconeus muscle", new tgBasicActuatorInfo(anconeusMuscleConfig));
    spec.addBuilder("brachiorodialis muscle", new tgBasicActuatorInfo(brachioradialisMuscleConfig));
    spec.addBuilder("support muscle", new tgBasicActuatorInfo(supportstringMuscleConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());

    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void FlemonsArmModel::step(double dt)
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

void FlemonsArmModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& FlemonsArmModel::getAllMuscles() const
{
    return allMuscles;
}
    
void FlemonsArmModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
