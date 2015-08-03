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
 * @file BigFoot.cpp
 * @brief Implementing a new design idea for BigPuppy's feet.
 * @author Dawn Hustig-Schultz
 * @date July 2015
 * @version 1.0.0
 * $Id$
 */


//This application
#include "BigFoot.h"

// This library
#include "core/tgModel.h"
#include "core/tgSimView.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <stdexcept>

BigFoot::BigFoot() :
tgModel() 
{
}

BigFoot::~BigFoot()
{
}

void BigFoot::setup(tgWorld& world)
{
    //Rod and Muscle configuration. Todo: make into structs in a namespace block. 

    const double density = 4.2/300.0; //Note: this needs to be high enough or things fly apart...
    const double radius = 0.5;
    const double rod_space = 10.0; //what was once v_size...
    const double friction = 0.5;
    const double rollFriction = 0.0;
    const double restitution = 0.0;
    const tgRod::Config rodConfig(radius, density, friction, rollFriction, restitution);

    const double radius2 = 0.15;
    const double density2 = 1;  // Note: This needs to be high enough or things fly apart...
    const tgRod::Config rodConfig2(radius2, density2);

    const double stiffness = 1000.0;
    const double damping = .01*stiffness;
    const double pretension = 0.0;

    const tgSpringCableActuator::Config stringConfig(stiffness, damping, pretension, false, 7000, 24);
    
    
    const double passivePretension = 700; // 5 N
    tgSpringCableActuator::Config muscleConfig(2000, 20, passivePretension);

    tgStructure foot;
    
    //Nodes. Todo: make into separate function
    foot.addNode(7,0,7);//0 : can't be less than 7, or structure spins. Weird, I know.
    foot.addNode(7,0,-7);//1 : ditto.
    foot.addNode(-7,0,-7);//2 : ditto.
    foot.addNode(-7,0,7);//3 : ditto. 
    foot.addNode(2,rod_space/2,0);//4
    foot.addNode(0,rod_space/2,-2);//5
    foot.addNode(-2,rod_space/2,0);//6
    foot.addNode(0,rod_space/2,2);//7

    //Rods. Todo: make into separate function
    foot.addPair(0,6,"rod");
    foot.addPair(1,7,"rod");
    foot.addPair(2,4,"rod");
    foot.addPair(3,5,"rod");

    //Muscles. Todo: make into separate function.
    foot.addPair(0,1,"muscle");
    foot.addPair(0,3,"muscle");
    foot.addPair(1,2,"muscle");
    foot.addPair(2,3,"muscle");
    foot.addPair(0,7,"muscle");
    foot.addPair(1,4,"muscle");
    foot.addPair(2,5,"muscle");
    foot.addPair(3,6,"muscle");
    foot.addPair(4,5,"muscle");
    foot.addPair(4,7,"muscle");
    foot.addPair(5,6,"muscle");
    foot.addPair(6,7,"muscle");

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(foot, spec);

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

void BigFoot::step(double dt)
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

const std::vector<tgSpringCableActuator*>& BigFoot::getAllActuators() const
{
    return allActuators;
}

void BigFoot::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
