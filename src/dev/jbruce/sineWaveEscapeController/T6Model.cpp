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
 * @file T6Model.cpp
 * @brief Contains the implementation of class T6Model.
 * $Id$
 */

// This module
#include "T6Model.h"
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

/*
 * helper arrays for node and rod numbering schema
 */
/*returns the number of the rod for a given node */
const int rodNumbersPerNode[13]={0,0,1,1,2,2,3,3,4,4,5,5,6};

/*returns the node that is at the other end of the given node */
const int otherEndOfTheRod[13]={6,7,8,4,3,11,0,1,2,10,9,5,12};

/*returns the node that is at the parallel rod
 * and at the same end of the given node
 */
const int parallelNode[13]={1,0,5,9,10,2,7,6,11,3,4,8,12};

namespace
{
    // Note: This current model of the SUPERball rod is 1.5m long by 4.5cm (0.045m) radius,
    // which is 1.684m*pi*(0.040m)^2 = 0.008545132m^3.
    // For SUPERball v1.5, mass = 3.5kg per strut, which density [kg/m^3] comes out to:
    // density = 3.45kg/0.008545132m^3 = 403.738642402kg/m^3 = 0.403738642402 kg/dm^3

    // see tgBaseString.h for a descripton of some of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1
    // Thus,
    // Rod Length = 17.00
    // Rod Radius = 0.40
    // density = 0.40374 (length is cubed, so decimal moves 3 places)

    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        double density;
        double radius;
        double stiffness_passive;
        double stiffness_active;
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
	 0.40374,   // density (kg / length^3)
     0.40,      // radius (length)
     998.25,    // stiffness_passive (kg / sec^2)
     3152.36,   // stiffness_active (kg / sec^2)
     50.0,      // damping (kg / sec)
     17.00,     // rod_length (length)
     17.00/4,   // rod_space (length)
     0.99,      // friction (unitless)
     0.01,      // rollFriction (unitless)
     0.0,       // restitution (?)
     0.0,         // pretension (force)
     true,		// History logging (boolean)
     100000,    // maxTens
     10000     // targetVelocity
#if (0)
     20000      // maxAcc
#endif
  };
} // namespace

T6Model::T6Model() : tgModel() 
{
}

T6Model::~T6Model()
{
}

void T6Model::addNodes(tgStructure& s)
{
    const double half_length = c.rod_length / 2;

    nodePositions.push_back(btVector3(-c.rod_space,  -half_length, 0));            // 0
    nodePositions.push_back(btVector3(-c.rod_space,   half_length, 0));            // 1
    nodePositions.push_back(btVector3( c.rod_space,  -half_length, 0));            // 2
    nodePositions.push_back(btVector3( c.rod_space,   half_length, 0));            // 3
    nodePositions.push_back(btVector3(0,           -c.rod_space,   -half_length)); // 4
    nodePositions.push_back(btVector3(0,           -c.rod_space,    half_length)); // 5
    nodePositions.push_back(btVector3(0,            c.rod_space,   -half_length)); // 6
    nodePositions.push_back(btVector3(0,            c.rod_space,    half_length)); // 7
    nodePositions.push_back(btVector3(-half_length, 0,            c.rod_space));   // 8
    nodePositions.push_back(btVector3( half_length, 0,            c.rod_space));   // 9
    nodePositions.push_back(btVector3(-half_length, 0,           -c.rod_space));   // 10
    nodePositions.push_back(btVector3( half_length, 0,           -c.rod_space));   // 11

    for(int i=0;i<nodePositions.size();i++)
	{
		s.addNode(nodePositions[i][0],nodePositions[i][1],nodePositions[i][2]);
	}
}

void T6Model::addRods(tgStructure& s)
{
    s.addPair( 0,  1, "rod");
    s.addPair( 2,  3, "rod");
    s.addPair( 4,  5, "rod");
    s.addPair( 6,  7, "rod");
    s.addPair( 8,  9, "rod");
    s.addPair(10, 11, "rod");
}

void T6Model::addMarkers(tgStructure &s) {
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

void T6Model::addMuscles(tgStructure& s)
{
	// Added ability to have two muscles in a structure.
	// Configuration currently is random, at the moment [27 June 2014]
    s.addPair(0, 4,  "muscle_active");
    s.addPair(0, 5,  "muscle_passive");
    s.addPair(0, 8,  "muscle_active");
    s.addPair(0, 10, "muscle_passive");

    s.addPair(1, 6,  "muscle_active");
    s.addPair(1, 7,  "muscle_passive");
    s.addPair(1, 8,  "muscle_passive");
    s.addPair(1, 10, "muscle_active");

    s.addPair(2, 4,  "muscle_passive");
    s.addPair(2, 5,  "muscle_active");
    s.addPair(2, 9,  "muscle_active");
    s.addPair(2, 11, "muscle_passive");

    s.addPair(3, 7,  "muscle_active");
    s.addPair(3, 6,  "muscle_passive");
    s.addPair(3, 9,  "muscle_passive");
    s.addPair(3, 11, "muscle_active");

    s.addPair(4, 10, "muscle_active");
    s.addPair(4, 11, "muscle_passive");

    s.addPair(5, 8,  "muscle_active");
    s.addPair(5, 9,  "muscle_passive");

    s.addPair(6, 10, "muscle_passive");
    s.addPair(6, 11, "muscle_active");

    s.addPair(7, 8,  "muscle_passive");
    s.addPair(7, 9,  "muscle_active");

}

void T6Model::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);

    tgBasicActuator::Config muscleConfig_passive(c.stiffness_passive, c.damping, c.pretension, c.hist, 
					    c.maxTens, c.targetVelocity);

    tgBasicActuator::Config muscleConfig_active(c.stiffness_active, c.damping, c.pretension, c.hist,
    					    c.maxTens, c.targetVelocity);
            
    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addMuscles(s);
    s.move(btVector3(0, 10, 0));

    // Add a rotation. This is needed if the ground slopes too much,
    // otherwise  glitches put a rod below the ground.
    btVector3 rotationPoint = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis = btVector3(0, 1, 0);  // y-axis
    double rotationAngle = M_PI/2;
    s.addRotation(rotationPoint, rotationAxis, rotationAngle);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle_passive", new tgBasicActuatorInfo(muscleConfig_passive));
    spec.addBuilder("muscle_active", new tgBasicActuatorInfo(muscleConfig_active));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
    // Not really sure how to use the find() function in tgModel.h
    passiveMuscles = tgCast::find<tgModel, tgBasicActuator>(tgTagSearch("muscle_passive"), getDescendants());
    activeMuscles = tgCast::find<tgModel, tgBasicActuator>(tgTagSearch("muscle_active"), getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);

    //map the rods and add the markers to them
	addMarkers(s);

	btVector3 location(0.0,28.0,0); // Start above ground (positive y)
	btVector3 rotation(0.0,0.6,0.8);
	btVector3 speed(0,0,0);
	this->moveModel(location,rotation,speed);
}

void T6Model::step(double dt)
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

void T6Model::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

// Return the center of mass of this model
// Pre-condition: This model has 6 rods
std::vector<double> T6Model::getBallCOM() {
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
    //std::cout<<"COM: (" << result[0] << ", " << result[1] << ", " << result[2] << ")\n";

    return result;
}

const std::vector<tgBasicActuator*>& T6Model::getAllMuscles() const
{
    return allMuscles;
}

const std::vector<tgBasicActuator*>& T6Model::getPassiveMuscles() const
{
    return passiveMuscles;
}

const std::vector<tgBasicActuator*>& T6Model::getActiveMuscles() const
{
    return activeMuscles;
}

const double T6Model::muscleRatio()
{
	return (c.stiffness_passive/c.stiffness_active);
	//return 2.5;
}
    
void T6Model::teardown()
{
    tgModel::teardown();
}

void T6Model::moveModel(btVector3 positionVector,btVector3 rotationVector,btVector3 speedVector) {
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
