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
 * @file UpperLimbTriangleModel.cpp
 * @brief Contains the implementation of class UpperLimbTriangleModel. See README
 * $Id$
 */

// This module
#include "UpperLimbTriangleModel.h"
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
        false,          // history (boolean)
        100000,         // maxTens
        10000,           // targetVelocity  
        1.0,            // mRad
        10.0,           // motorFriction
        1.0,            // motorInertia
        false           // backDrivable (boolean)
    };
} // namespace

UpperLimbTriangleModel::UpperLimbTriangleModel() : tgModel() {}

UpperLimbTriangleModel::~UpperLimbTriangleModel() {}

void UpperLimbTriangleModel::addNodes(tgStructure& s) {
    const double scale = 0.5;
    const double bone_scale = 0.3;
    const size_t nNodes = 28; //TODO: Change to variable, incremented by calling sub-structure node adding functions (i.e. one for olecranon, one for humerus, etc.)
    
    // Average Adult Male Measurements with scale
    // Lengths are in mm
    const double b = 334 * scale * bone_scale; //ulna length
    const double c = 265 * scale * bone_scale; //humerus length
    const double g = 17 * scale; //ulna proximal width
    const double e = g/4;
    const double f = c+7*e;

    //Format: (x, z, y)
    // olecranon (0-3)
    nodePositions.push_back(btVector3(0, 0, 0));
    nodePositions.push_back(btVector3(-g, g, 0));
    nodePositions.push_back(btVector3(g, g, 0));
    nodePositions.push_back(btVector3(g, -g, 0));
    
    // ulna-radius (4-7)
    nodePositions.push_back(btVector3(3*e, 0, g));
    nodePositions.push_back(btVector3(3*e, 0, -g));
    nodePositions.push_back(btVector3(7*e, 0, 0));
    nodePositions.push_back(btVector3(b+7*e, 0, 0));

    // humerus (8-13)
    nodePositions.push_back(btVector3(0, 3*e, g));
    nodePositions.push_back(btVector3(0, 3*e, -g));
    nodePositions.push_back(btVector3(0, 7*e, 0));
    nodePositions.push_back(btVector3(g/2, 5*f/6, g/2));
    nodePositions.push_back(btVector3(0, f+3*g/2, 5*g/6));
    nodePositions.push_back(btVector3(g*2, f+3*g/2, 5*g/6));

    // clavicle (14-15)
    nodePositions.push_back(btVector3(g, f+2*g, 0));
    nodePositions.push_back(btVector3(2*g, f+2*g, 4*g));

    // scapula (16-18)                                  
    nodePositions.push_back(btVector3(-g/2, f+2*g, -g));
    nodePositions.push_back(btVector3(-g/2, f+2*g, 2*g));
    nodePositions.push_back(btVector3(-g/2, f-g, 2*g));

    // added 9/3/15
    // Scapula tetrahedron: 19 
    nodePositions.push_back(btVector3(g, f+(3*g/2), g/3));

    // Humerus tetrahedron: 20
    nodePositions.push_back(btVector3(g, f+9*g/16, 3*g/2));

    //added object 2/27/16
    // Added Object: 21
    nodePositions.push_back(btVector3(b+7*e+1 + 4, 50, 28));
    // Added Object: 22
    nodePositions.push_back(btVector3(1+b+7*e +4, 50, 25));
    // Added Object: 23
    nodePositions.push_back(btVector3(1+b+7*e +4, 47, 25));
    // Added Object: 24
    nodePositions.push_back(btVector3(1+b+7*e +4, -47, 25));
    // Added Object: 25
    nodePositions.push_back(btVector3(1+b+7*e +4, -50, 28));
    // Added Object: 26
    nodePositions.push_back(btVector3(1+b+7*e +4, -50, 25));

    //end effector (27)
    nodePositions.push_back(btVector3(b+7*e, 1, 0));


    for(size_t i=0;i<nNodes;i++) {
		s.addNode(nodePositions[i][0],nodePositions[i][1],nodePositions[i][2]);
    }
}
                  
void UpperLimbTriangleModel::addRods(tgStructure& s) {   
    // olecranon
    s.addPair(0, 1, "olecranon bone");
    s.addPair(0, 2, "olecranon bone");
    s.addPair(0, 3, "olecranon bone");

    // ulna and radius
    s.addPair(4, 6,  "radiusulna bone");
    s.addPair(5, 6,  "radiusulna bone");
    s.addPair(6, 7,  "radiusulna bone");
    s.addPair(7, 27,  "eeRod endeffector");


    // humerus
    s.addPair(8, 10, "humerus bone");
    s.addPair(9, 10, "humerus bone"); 
    s.addPair(10, 11, "humerus bone");
    s.addPair(11, 12, "humerus bone");
    s.addPair(11, 13, "humerus bone");
    s.addPair(12, 13, "humerus bone");
    s.addPair(11, 20, "humerus bone");
    s.addPair(12, 20, "humerus bone");
    s.addPair(20, 13, "humerus bone");

    // clavicle
    s.addPair(14, 15, "clavicle massless");

    // scapula
    s.addPair(16, 17, "scapula massless");
    s.addPair(16, 18, "scapula massless");
    s.addPair(17, 18, "scapula massless");

    //added 9/3/15
    s.addPair(16, 19, "scapula massless");
    s.addPair(17, 19, "scapula massless");
    s.addPair(18, 19, "scapula massless");

    //added 2/27/16
    s.addPair(23, 24, "humerus bone");
    s.addPair(22, 21, "scapula massless");
    s.addPair(26, 25, "scapula massless");    
}

void UpperLimbTriangleModel::addMuscles(tgStructure& s) {
    const std::vector<tgStructure*> children = s.getChildren();

    // ulna-radius to olecranon
    s.addPair(6, 2, "olecranon muscle");
    s.addPair(6, 3, "olecranon muscle");

    s.addPair(4, 0, "olecranon muscle");
    s.addPair(4, 2, "olecranon muscle");
    s.addPair(4, 3, "olecranon muscle");
    s.addPair(4, 8, "right anocneus  muscle");
    
    s.addPair(5, 0, "olecranon muscle");
    s.addPair(5, 2, "olecranon muscle");
    s.addPair(5, 3, "olecranon muscle");
    s.addPair(5, 9, "left anconeus muscle");
    
    // humerus to olecranon
    s.addPair(8, 0, "olecranon muscle");
    s.addPair(8, 1, "olecranon muscle");
    s.addPair(8, 2, "olecranon muscle");
    
    s.addPair(9, 0, "olecranon muscle");
    s.addPair(9, 1, "olecranon muscle");
    s.addPair(9, 2, "olecranon muscle");
    
    s.addPair(10, 1, "olecranon muscle");
    s.addPair(10, 2, "brachioradialis muscle");
/*
    // humerus to clavicle
    s.addPair(12, 14, "olecranon muscle"); //TODO: Change muscle names in entire function
    s.addPair(13, 14, "olecranon muscle");

    // humerus to scapula
    s.addPair(12, 16, "olecranon muscle");
    s.addPair(13, 16, "olecranon muscle");
*/
    // humerus to clavicle
    s.addPair(13, 14, "olecranon muscle");
    
    // humerus to scapula
    s.addPair(20, 18, "brachioradialis muscle");
    s.addPair(12, 16, "brachioradialis muscle");
    s.addPair(13, 17, "brachioradialis muscle");
    s.addPair(13, 19, "brachioradialis muscle");
    
    s.addPair(23, 21, "olecranon muscle");
    s.addPair(24, 25, "olecranon muscle");
}
 
/*
void UpperLimbTriangleModel::addMarkers(tgStructure &s) {
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
 
void UpperLimbTriangleModel::setup(tgWorld& world) {
    const tgRod::Config boneConfig(cRod.radius, cRod.density, cRod.friction, cRod.rollFriction, cRod.restitution);
    const tgRod::Config boneConfigMassless(cRod.radius, 0.00/*c.density*/, cRod.friction, cRod.rollFriction, cRod.restitution);
const tgRod::Config eeConfig(cRod.radius, cRod.density/4/*c.density*/, cRod.friction, cRod.rollFriction, cRod.restitution);
    /// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate

    tgBasicActuator::Config olecranonMuscleConfig(cCable.stiffness, cCable.damping, cCable.pretension_olecranon, 
                                                  cCable.history, cCable.maxTens, cCable.targetVelocity);
    tgBasicActuator::Config anconeusMuscleConfig(cCable.stiffness, cCable.damping, cCable.pretension_anconeus, 
                                                 cCable.history, cCable.maxTens, cCable.targetVelocity);
    tgBasicActuator::Config brachioradialisMuscleConfig(cCable.stiffness, cCable.damping, cCable.pretension_brachioradialis, 
                                                        cCable.history, cCable.maxTens, cCable.targetVelocity);
            
    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addMuscles(s);
    
    // Move the arm out of the ground
    btVector3 offset(0.0, 60.0, 0.0);
    s.move(offset);
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("massless", new tgRodInfo(boneConfigMassless));
    spec.addBuilder("bone", new tgRodInfo(boneConfig));
    spec.addBuilder("olecranon muscle", new tgBasicActuatorInfo(olecranonMuscleConfig));
    spec.addBuilder("anconeus muscle", new tgBasicActuatorInfo(anconeusMuscleConfig));
    spec.addBuilder("brachioradialis muscle", new tgBasicActuatorInfo(brachioradialisMuscleConfig));
    spec.addBuilder("eeRod", new tgRodInfo(eeConfig));
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

void UpperLimbTriangleModel::step(double dt)
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

void UpperLimbTriangleModel::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& UpperLimbTriangleModel::getAllMuscles() const
{
    return allMuscles;
}
    
void UpperLimbTriangleModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}

