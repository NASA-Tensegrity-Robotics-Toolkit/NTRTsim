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
 * @file tgBoxAnchorDebugModel.cpp
 * @brief Contains the implementation of class tgBoxAnchorDebugModel.
 * $Id$
 */

// This module
#include "tgBoxAnchorDebugModel.h"
// This library
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
        double rodLength;
        double stiffness;
        double damping;  
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        bool   hist;
        double maxTens;
        double targetVelocity;
        bool moveCablePointAToEdge;
        bool moveCablePointBToEdge;
    } c =
   {
     0.688,    // density (kg / length^3)
     3.0,     // radius (length)
     2.0,      // rodLength (length)
     613.0,   // stiffness (kg / sec^2) was 1500
     200.0,    // damping (kg / sec)
     0.99,      // friction (unitless)
     0.01,     // rollFriction (unitless)
     0.0,      // restitution (?)
     200.0,        // pretension -> set to 4 * 613, the previous value of the rest length controller
     0,			// History logging (boolean)
     100000,   // maxTens
     10000,    // targetVelocity
     false,    // moveCablePointAToEdge
     false,     // moveCablePointBToEdge

     // Use the below values for earlier versions of simulation.
     // 1.006,    
     // 0.31,     
     // 300000.0, 
     // 3000.0,   
     // 15.0,     
     // 7.5,      
  };
} // namespace

// Constructor does nothing. Everything happens in setup.
tgBoxAnchorDebugModel::tgBoxAnchorDebugModel() : tgModel() 
{
}

// Destructor does nothing. Everythign happens in teardown.
tgBoxAnchorDebugModel::~tgBoxAnchorDebugModel()
{
}

void tgBoxAnchorDebugModel::addNodes(tgStructure& s)
{
  s.addNode(0, 0, 0);            // 0
  s.addNode(0, c.rodLength, 0);            // 1
  s.addNode(0, 2 * c.rodLength, 0);            // 2
  s.addNode(0, 3 * c.rodLength, 0);            // 3
}

void tgBoxAnchorDebugModel::addRods(tgStructure& s)
{
    s.addPair( 0,  1, "rod");
    s.addPair( 2,  3, "rod");
}

void tgBoxAnchorDebugModel::addActuators(tgStructure& s)
{
    s.addPair(1, 2,  "muscle");
}

void tgBoxAnchorDebugModel::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);
    
    /// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate
    tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension,
					 c.hist, c.maxTens, c.targetVelocity);
    // Since the boolean flags for moving the anchor attachment points
    // are not next in line in the Config constructor, assign them explicitly
    // after the config struct has been constructed:
    muscleConfig.moveCablePointAToEdge = c.moveCablePointAToEdge;
    muscleConfig.moveCablePointBToEdge = c.moveCablePointBToEdge;
            
    // Start creating the structure
    tgStructure s;
    addNodes(s);

    /*
    // DEBUGGING
    // Check: did the nodes move?
    std::cout << "Nodes in the structure after addNodes: " <<
      s.getNodes() << std::endl;
    */
    
    addRods(s);
    addActuators(s);

    /*
    // DEBUGGING
    // Check: did the nodes move?
    std::cout << "Nodes in the structure after addPairs: " <<
      s.getNodes() << std::endl;
    // Check: did the pairs move?
    std::cout << "Pairs in the structure after addPairs: " << std::endl;
    // need to iterate through the std::vector of tgPair objects
    std::vector<tgPair> pairsAfterAdd = s.getPairs().getPairs();
    for( size_t i = 0; i < pairsAfterAdd.size(); i++ ) {
      // Print the pair.
      std::cout << pairsAfterAdd[i] << ", from and to: " <<
	pairsAfterAdd[i].getFrom() << " , " << pairsAfterAdd[i].getTo() << std::endl;
    }
    */

    // Move the whole structure up a bit.
    s.move(btVector3(0, 5, 0));

    /*
    // DEBUGGING
    // Check: did the nodes move?
    std::cout << "Nodes in the structure after move: " <<
      s.getNodes() << std::endl;
    // Check: did the pairs move?
    std::cout << "Pairs in the structure after move: " << std::endl;
    // need to iterate through the std::vector of tgPair objects
    std::vector<tgPair> pairsAfterMove = s.getPairs().getPairs();
    for( size_t i = 0; i < pairsAfterMove.size(); i++ ) {
      // Print the pair.
      std::cout << pairsAfterMove[i] << ", from and to: " <<
	pairsAfterMove[i].getFrom() << " , " << pairsAfterMove[i].getTo() <<
	std::endl;
    }
    */

    // Add a rotation. This is needed if the ground slopes too much,
    // otherwise  glitches put a rod below the ground.
    /*btVector3 rotationPoint = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis = btVector3(0, 1, 0);  // y-axis
    double rotationAngle = M_PI/2;
    s.addRotation(rotationPoint, rotationAxis, rotationAngle);
    */

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
}

void tgBoxAnchorDebugModel::step(double dt)
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

void tgBoxAnchorDebugModel::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& tgBoxAnchorDebugModel::getAllActuators() const
{
    return allActuators;
}
    
void tgBoxAnchorDebugModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
