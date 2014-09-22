/*
 * Copyright © 2014, United States Government, as represented by the
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
 * @file Escape_T6Model.cpp
 * @brief Contains the implementation of class Escape_T6Model.
 * $Id$
 */

// This module
#include "Escape_T6Model.h"
// This library
#include "core/tgLinearString.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgLinearStringInfo.h"
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
        bool   history;
        double damping;
        double rod_length;
        double rod_space;    
        double friction;
        double rollFriction;
        double restitution;
        double rotation;  
        double maxTens;
        double targetVelocity;
        double maxAcc;
    } c =
    {
        0.825,    // density (kg / length^3)
        0.31,     // radius (length)
        3000.0,   // stiffness (kg / sec^2)
        true,     // history (record?)
        200.0,    // damping (kg / sec)
        15.0,     // rod_length (length)
        7.5,      // rod_space (length)
        1.0,      // friction (unitless)
        0.01,     // rollFriction (unitless)
        0.2,      // restitution (?)
        0,        // rotation
        500000,   // maxTens (dN)           // Ideally as low as 5000
        26000,    // targetVelocity (dm/s)  // Ideally as low as 2.6
        20000     // maxAcc

            // Use the below values for earlier versions of simulation.
            // 1.006,    
            // 0.31,     
            // 300000.0, 
            // 3000.0,   
            // 15.0,     
            // 7.5,      
    };
} // namespace

/*
 * helper arrays for node and rod numbering schema
 */
/*returns the number of the rod for a given node */
const int rodNumbersPerNode[13]={0,1,2,3,3,4,0,1,2,5,5,4,6};

/*returns the node that is at the other end of the given node */
const int otherEndOfTheRod[13]={6,7,8,4,3,11,0,1,2,10,9,5,12};

/*returns the node that is at the parallel rod
 * and at the same end of the given node
 */
const int parallelNode[13]={1,0,5,9,10,2,7,6,11,3,4,8,12};

Escape_T6Model::Escape_T6Model() : tgModel() {}

Escape_T6Model::~Escape_T6Model() {}

//Node numbers seen from Front
// -----0-------1------
// ---------2----------
// 3------------------4
// ---------5----------
// -----6-------7------
//
//Node numbers seen from Back
// -----0-------1------
// ---------8----------
// 9-----------------10
// ---------11---------
// -----6-------7------
//
 
void Escape_T6Model::setup(tgWorld& world) {
    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
            c.rollFriction, c.restitution);

    tgLinearString::Config muscleConfig(c.stiffness, c.damping, 
            c.history, c.rotation, c.maxTens, c.targetVelocity, 
            c.maxAcc);

    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addMuscles(s);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgLinearStringInfo(muscleConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgLinearString> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);

    //map the rods and add the markers to them
    addMarkers(s);

    btVector3 location(0.0,40.0,0); // Start above ground (positive y)
    btVector3 rotation(0.0,0.6,0.8);
    btVector3 speed(0,0,0);
    this->moveModel(location,rotation,speed);
}
 
void Escape_T6Model::step(double dt) {
    // Precondition
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    }
    else {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void Escape_T6Model::onVisit(tgModelVisitor& r) {
    tgModel::onVisit(r);
}
                      
void Escape_T6Model::teardown() {
    notifyTeardown();
    tgModel::teardown();
}
                      
const std::vector<tgLinearString*>& Escape_T6Model::getAllMuscles() const {
    return allMuscles;
}
                                                     
// Return the center of mass of this model
// Pre-condition: This model has 6 rods
std::vector<double> Escape_T6Model::getBallCOM() {   
    std::vector <tgRod*> rods = find<tgRod>("rod");
    assert(!rods.empty());

    btVector3 ballCenterOfMass(0, 0, 0);
    double ballMass = 0.0; 
    for (std::size_t i = 0; i < rods.size(); i++) {   
        const tgRod* const rod = rods[i];
        assert(rod != NULL);
        const double rodMass = rod->mass();
        const btVector3 rodCenterOfMass = rod->centerOfMass();
        ballCenterOfMass += rodCenterOfMass * rodMass;
        ballMass += rodMass;
    }

    assert(ballMass > 0.0);
    ballCenterOfMass /= ballMass;

    // Copy to the result std::vector
    std::vector<double> result(3);
    for (size_t i = 0; i < 3; ++i) { result[i] = ballCenterOfMass[i]; }

    return result;
}
                                     
void Escape_T6Model::addNodes(tgStructure& s) {
    const double half_length = c.rod_length / 2;

    nodePositions.push_back(btVector3(-half_length, c.rod_space, 0));            // 0
    nodePositions.push_back(btVector3( half_length, c.rod_space, 0));            // 1
    nodePositions.push_back(btVector3(0,            half_length, -c.rod_space)); // 2
    nodePositions.push_back(btVector3(-c.rod_space, 0,           -half_length)); // 3
    nodePositions.push_back(btVector3( c.rod_space, 0,           -half_length)); // 4
    nodePositions.push_back(btVector3(0,            -half_length, -c.rod_space));// 5
    nodePositions.push_back(btVector3(-half_length, -c.rod_space, 0));           // 6
    nodePositions.push_back(btVector3( half_length, -c.rod_space, 0));           // 7
    nodePositions.push_back(btVector3(0,            half_length,  c.rod_space)); // 8
    nodePositions.push_back(btVector3(-c.rod_space, 0,            half_length)); // 9
    nodePositions.push_back(btVector3( c.rod_space, 0,            half_length)); // 10
    nodePositions.push_back(btVector3(0,            -half_length, c.rod_space)); // 11

    for(int i=0;i<nodePositions.size();i++)
    {
        s.addNode(nodePositions[i][0],nodePositions[i][1],nodePositions[i][2]);
    }
}

void Escape_T6Model::addRods(tgStructure& s) {
    s.addPair( 0,  6, "r1 rod");
    s.addPair( 1,  7, "r2 rod");
    s.addPair( 2,  8, "r3 rod");
    s.addPair( 3,  4, "r4 rod");
    s.addPair( 5, 11, "r5 rod");
    s.addPair( 9, 10, "r6 rod");
}

void Escape_T6Model::addMarkers(tgStructure &s) {
    const int nNodes = 12; // 2*nRods
    std::vector <tgRod*> rods=find<tgRod>("rod");

    for(int i=0;i<nNodes;i++) {
        const btRigidBody* bt = rods[rodNumbersPerNode[i]]->getPRigidBody();
        btTransform inverseTransform = bt->getWorldTransform().inverse();
        btVector3 pos = inverseTransform * (nodePositions[i]);
        abstractMarker tmp=abstractMarker(bt,pos,btVector3(0.08*i,1.0 - 0.08*i,.0),i);
        this->addMarker(tmp);
    }
}

/** 
 * Defines muscles in the structure by their end nodes 
 * as well as the cluster to which they belong
 * A cluster is any three muscles that form a triangle 
 * (with nodes as vertices)
 */
void Escape_T6Model::addMuscles(tgStructure& s) {
    // Cluster 1
    s.addPair(0, 3, "muscle cluster1");
    s.addPair(3, 2, "muscle cluster1");
    s.addPair(2, 0, "muscle cluster1");

    // Cluster 2
    s.addPair(4, 5, "muscle cluster2");
    s.addPair(5, 7, "muscle cluster2");
    s.addPair(7, 4, "muscle cluster2");

    // Cluster 3
    s.addPair(1, 8, "muscle cluster3");
    s.addPair(8, 10, "muscle cluster3");
    s.addPair(10, 1, "muscle cluster3");

    //Cluster 4
    s.addPair(9, 11, "muscle cluster4");
    s.addPair(11, 6, "muscle cluster4");
    s.addPair(6, 9, "muscle cluster4");

    // Cluster 5
    s.addPair(1, 2, "muscle cluster5");
    s.addPair(2, 4, "muscle cluster5");
    s.addPair(4, 1, "muscle cluster5");

    // Cluster 6
    s.addPair(3, 5, "muscle cluster6");
    s.addPair(5, 6, "muscle cluster6");
    s.addPair(6, 3, "muscle cluster6");

    // Cluster 7
    s.addPair(0, 8, "muscle cluster7");
    s.addPair(8, 9, "muscle cluster7");
    s.addPair(0, 9, "muscle cluster7");

    // Cluster 8
    s.addPair(11, 7, "muscle cluster8");
    s.addPair(7, 10, "muscle cluster8");
    s.addPair(10, 11, "muscle cluster8");
}

void Escape_T6Model::moveModel(btVector3 positionVector,btVector3 rotationVector,btVector3 speedVector) {
    std::vector<tgRod *> rods=find<tgRod>("rod");

    btQuaternion initialRotationQuat;
    initialRotationQuat.setEuler(rotationVector[0],rotationVector[1],rotationVector[2]);
    btTransform initialTransform;
    initialTransform.setIdentity();
    initialTransform.setRotation(initialRotationQuat);
    initialTransform.setOrigin(positionVector);
    for(int i=0;i<rods.size();i++) {
        rods[i]->getPRigidBody()->setLinearVelocity(speedVector);
        rods[i]->getPRigidBody()->setWorldTransform(initialTransform * rods[i]->getPRigidBody()->getWorldTransform());
    }
}

