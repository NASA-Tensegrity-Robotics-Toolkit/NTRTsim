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
 * @file SecondKneeModel.cpp
 * @brief Contains the definition of the members of the class SecondKneeModel.
 * $Id$
 */

// This module
#include "SecondKneeModel.h"
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
        double pretension;
        double triangle_length;
        double triangle_height;
        double Knee_height;  
    } c =
   {
       0.2,     // density (mass / length^3)
       0.31,     // radius (length)
       1000.0,   // stiffness (mass / sec^2)
       10.0,     // damping (mass / sec)
       500.0,     // pretension (mass * length / sec^2)
       10.0,     // triangle_length (length)
       10.0,     // triangle_height (length)
       20.0,     // Knee_height (length)
  };
} // namespace

SecondKneeModel::SecondKneeModel() :
tgModel() 
{
}

SecondKneeModel::~SecondKneeModel()
{
}

void SecondKneeModel::addNodes(tgStructure& s,
                            double edge,
                            double width,
                            double height)
{
//tibia and fibia (Cross Beams)
    //bottom origin
	s.addNode(0,0,0);//0
    // bottom right
    s.addNode(-edge / 2.0, 0, 0); // 1
    // bottom left
    s.addNode( edge / 2.0, 0, 0); // 2
    // bottom front
    s.addNode(0, 0, width/1.5); // 3
    // bottom back
    s.addNode(0, 0, -width/1.5); //4
    //top origin
	s.addNode(0, height, 0);//5
    // bottom right
    s.addNode(-edge / 2.0, height, 0); // 6
    // bottom left
    s.addNode( edge / 2.0, height, 0); // 7
    // bottom front
    s.addNode(0, height, width/1.5); // 8
    // bottom back
    s.addNode(0, height, -width/1.5); //9
    

//humerus
	//bottom origin
    s.addNode(0, height+2, 0); //10
    // bottom right
    s.addNode(-edge/1.5, height+2, 0); // 11
    // bottom left
    s.addNode( edge/1.5, height+2, 0); // 12
    // bottom front
    s.addNode(0, height+2, width/2); // 13
    // bottom back
    s.addNode(0, height+2, -width/2);// 14
	//top origin
    s.addNode(0, height*2, 0); //15
    // top right
    s.addNode(-edge/1.5, height*2, 0); // 16
    // top left
    s.addNode( edge/1.5, height*2, 0); // 17
    // top front
    s.addNode(0, height*2, width/2); // 18
    // top back
    s.addNode(0, height*2, -width/2);//19

}

void SecondKneeModel::addRods(tgStructure& s)
{
//fibia
	//Bottom
    s.addPair( 0,  1, "rod");
    s.addPair( 0,  2, "rod");
    s.addPair( 0,  3, "rod");
    s.addPair( 0, 4, "rod");
    s.addPair(0, 5, "rod"); 
	//Top
    s.addPair( 5,  6, "rod");
    s.addPair( 5,  7, "rod");
    s.addPair( 5,  8, "rod");
    s.addPair( 5,  9, "rod"); 


//tibia
	//Bottom	
	s.addPair( 10, 11, "rod");
	s.addPair( 10, 12, "rod");
	s.addPair( 10, 13, "rod");
	s.addPair( 10, 14, "rod");
	s.addPair( 10, 15, "rod");

	//Top
	s.addPair(15, 16, "rod");
	s.addPair(15, 17, "rod");
	s.addPair(15, 18, "rod");
	s.addPair(15, 19, "rod");
}

void SecondKneeModel::addMuscles(tgStructure& s)
{
//Tibia and Fibia
	s.addPair(3, 12, "muscle");//Calve
	s.addPair(4, 12, "muscle");
	s.addPair(1, 11, "muscle");//Frontal calve
//Joint
	s.addPair(8, 13, "muscle");//ACL
	s.addPair(9, 14, "muscle");//PCL
	s.addPair(6, 16, "muscle");//Patella
	
//Humerus
	s.addPair(8, 17, "muscle");//Hamstring
	s.addPair(9, 17, "muscle");
	s.addPair(8, 16, "muscle");//Quads
	s.addPair(9, 16, "muscle");   
}

void SecondKneeModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config rodConfig(c.radius, c.density);
    const tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension);
    
    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the structure
    addNodes(s, c.triangle_length, c.triangle_height, c.Knee_height);
    
    // Add rods to the structure
    addRods(s);
    
    // Add muscles to the structure
    addMuscles(s);
    
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 10, 0));
    
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
    allActuators = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    
    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void SecondKneeModel::step(double dt)
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

void SecondKneeModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& SecondKneeModel::getAllActuators() const
{
    return allActuators;
}
    
void SecondKneeModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
