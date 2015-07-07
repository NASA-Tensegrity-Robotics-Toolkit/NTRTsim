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
 * @file ScarrArmModel.cpp
 * @brief Contains the implementation of class ScarrArmModel. See README
 * $Id$
 */

// This module
#include "ScarrArmModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "core/abstractMarker.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgKinematicContactCableInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>

namespace {
    // see tgBaseString.h for a descripton of some of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    const struct ConfigRod {
        double density;
        double radius;
        double rod_length;
        double rod_space;    
        double friction;
        double rollFriction;
        double restitution;
    } cRod = {
        0.05,    // density (kg / length^3)
        0.8,     // radius (length)
        15.0,     // rod_length (length)
        7.5,      // rod_space (length)
        1.0,      // friction (unitless)
        0.01,     // rollFriction (unitless)
        0.2      // restitution (?)
    };

    const struct ConfigCable {
        double elasticity;
        double damping;
        double stiffness;
        double pretension_olecranon;
        double pretension_anconeus;
        double pretension_brachioradialis;
        double pretension_support;
        bool   history;  
        double maxTens;
        double targetVelocity; 
        double mRad;
        double motorFriction;
        double motorInertia;
        bool backDrivable;
    } cCable = {
        1000.0,         // elasticity
        200.0,          // damping (kg/s)
        3000.0,         // stiffness (kg / sec^2)
        3000.0/1,       // pretension_olecranon (force), stiffness/initial length
        3000.0/15.55,   // pretension_anconeus (force), stiffness/initial length
        3000.0/262,     // pretension_brachioradialis (force), stiffness/initial length 
        30000.0/1,      // pretension_support (force), stiffness/initial length 
        false,          // history (boolean)
        100000,         // maxTens
        10000,           // targetVelocity  
        1.0,            // mRad
        10.0,           // motorFriction
        1.0,            // motorInertia
        false           // backDrivable (boolean)
    };
} // namespace

ScarrArmModel::ScarrArmModel() : tgModel() {}

ScarrArmModel::~ScarrArmModel() {}

void ScarrArmModel::addNodes(tgStructure& s) {
    const double scale = 0.5;
    const double bone_scale = 0.3;
    const size_t nNodes = 15 + 2; //2 for massless rod
    
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
    nodePositions.push_back(btVector3(g, 0, 0));
    nodePositions.push_back(btVector3(0, -g, 0));
    nodePositions.push_back(btVector3(-a/2, 0, 0));
    nodePositions.push_back(btVector3(0, 0, g));
    nodePositions.push_back(btVector3(0, 0, -g));
    nodePositions.push_back(btVector3(0, g, 0));
    nodePositions.push_back(btVector3(0, c, 0));
    nodePositions.push_back(btVector3(x, z, 0));
    nodePositions.push_back(btVector3(b+a/2, -g, 0));
    nodePositions.push_back(btVector3(0, c+2, f));
    nodePositions.push_back(btVector3(0, c+2, -f));

    //Added 6/17/15
    nodePositions.push_back(btVector3(a/2, -2*g, 0));

    //ulna and radius
    nodePositions.push_back(btVector3(3*a/2, -g, 0));
    nodePositions.push_back(btVector3(3*a/4, -g, g));
    nodePositions.push_back(btVector3(3*a/4, -g, -g));

    nodePositions.push_back(btVector3(f, c+2, 0));
    nodePositions.push_back(btVector3(-f, c+2, 0));

    for(size_t i=0;i<nNodes;i++) {
		s.addNode(nodePositions[i][0],nodePositions[i][1],nodePositions[i][2]);
    }
}
                  
void ScarrArmModel::addRods(tgStructure& s) {   
    // ulna and radius
    s.addPair(8,  12,  "rod");
    s.addPair(12,  14,  "rod");
    s.addPair(12,  13,  "rod");

    //olecranon
    s.addPair(0,  1,  "rod");
    s.addPair(1,  2,  "rod");
    s.addPair(1, 11,  "rod");

    // humerus
    s.addPair(3,  5,  "humerus massless");
    s.addPair(4,  5,  "humerus massless"); 
    s.addPair(5,  6,  "humerus massless");
}

void ScarrArmModel::addMuscles(tgStructure& s) {
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
}
 
/*
void ScarrArmModel::addMarkers(tgStructure &s) {
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
 
void ScarrArmModel::setup(tgWorld& world) {
    const tgRod::Config rodConfig(cRod.radius, cRod.density, cRod.friction, cRod.rollFriction, cRod.restitution);
    const tgRod::Config rodConfigMassless(cRod.radius, 0.00/*c.density*/, cRod.friction, cRod.rollFriction, cRod.restitution);
    /// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate

    tgBasicActuator::Config olecranonMuscleConfig(cCable.stiffness, cCable.damping, cCable.pretension_olecranon, 
                                                  cCable.history, cCable.maxTens, cCable.targetVelocity);
    tgBasicActuator::Config anconeusMuscleConfig(cCable.stiffness, cCable.damping, cCable.pretension_anconeus, 
                                                 cCable.history, cCable.maxTens, cCable.targetVelocity);
    tgBasicActuator::Config brachioradialisMuscleConfig(cCable.stiffness, cCable.damping, cCable.pretension_brachioradialis, 
                                                        cCable.history, cCable.maxTens, cCable.targetVelocity);
    tgBasicActuator::Config supportstringMuscleConfig(cCable.stiffness, cCable.damping, cCable.pretension_support, 
                                                      cCable.history, cCable.maxTens, cCable.targetVelocity);
            
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

void ScarrArmModel::step(double dt)
{
    // Precondition
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    } else {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void ScarrArmModel::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& ScarrArmModel::getAllMuscles() const
{
    return allMuscles;
}
    
void ScarrArmModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}

