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
 * @file AppTgBoxAnchorDebugDemo.cpp
 * @brief Contains the definition function main() for the demo of the
 * debugging application for tgBox's anchor positions.
 * @author Drew Sabelhaus
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// This application
#include "tgBoxAnchorDebugModel.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgRod.h" // for hinge hack
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "sensors/tgDataLogger2.h"
#include "sensors/tgRodSensorInfo.h"
#include "sensors/tgSpringCableActuatorSensorInfo.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
#include "core/tgWorldBulletPhysicsImpl.h" // for hinge hack
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h" // for hinge hack
#include "BulletDynamics/Dynamics/btDynamicsWorld.h" // for hinge hack
// The C++ Standard Library
#include <iostream>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "AppTgBoxAnchorDebugDemo" << std::endl;

    // First create the ground and world
    
    // Determine the angle of the ground in radians. All 0 is flat
    const double yaw = 0.0;
    //const double pitch = M_PI/15.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    //tgBoxGround* ground = new tgBoxGround(groundConfig);
    tgBoxGround* ground = new tgBoxGround();
    
    const tgWorld::Config config(98.1); // gravity, cm/sec^2  Use this to adjust length scale of world.
        // Note, by changing the setting below from 981 to 98.1, we've
        // scaled the world length scale to decimeters not cm.

    tgWorld world(config, ground);

    // Second create the view
    const double timestep_physics = 0.001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    tgModel* const myModel = new tgBoxAnchorDebugModel();

    // Finally, add out model to the simulation
    simulation.addModel(myModel);

    // Test: can we play with the underlying bullet physics stuff (e.g. constraints)
    // from the app file? Super hacky...
    std::vector<tgModel*> all_children = myModel->getDescendants();
    // Pick out the tgRods:
    std::vector<tgRod*> allRods = tgCast::filter<tgModel, tgRod>(all_children);
        for (size_t i = 0; i < allRods.size(); i++)
    {
      std::cout << "Inside App: rod number " << i << ": " << std::endl;
      std::cout << "tags: " << allRods[i]->getTags() << std::endl;
      std::cout << allRods[i]->toString() << std::endl;
    }
    // Test of the btHingeConstraint.
    // First, pick out the Bullet rigid bodies of the two rods.
    btRigidBody* rodA_rb = allRods[0]->getPRigidBody();
    btRigidBody* rodB_rb = allRods[1]->getPRigidBody();
        // Create the hinge constraint
    // Constructor is: 2 x btRigidBody, 4 x btVector3, 1 x bool.
    btHingeConstraint* rotHinge =
      new btHingeConstraint(*rodA_rb, *rodB_rb, btVector3(0, 3, 0),
			    btVector3(0, 0, 0), btVector3(0, 1, 0),
			    btVector3(0, 1, 0), false);
    // Next, we need to get a reference to the Bullet Physics world.
    //tgWorld simWorld = simulation.getWorld();
    tgWorldImpl& impl = world.implementation();
    tgWorldBulletPhysicsImpl& bulletWorld = static_cast<tgWorldBulletPhysicsImpl&>(impl);
    btDynamicsWorld* btWorld = &bulletWorld.dynamicsWorld();
    // Add the hinge constraint to the world.
    btWorld->addConstraint(rotHinge);
    
    // For the sensors:
    // A string prefix for the filename
    std::string log_filename = "~/NTRTsim_logs/AppTgBoxAnchorDebugDemo";
    // First, create the data manager
    tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename);
    //std::cout << myDataLogger->toString() << std::endl;
    // Then, add the model to the data logger
    myDataLogger->addSenseable(myModel);
    // Create sensor infos for all the types of sensors that the data logger
    // will create.
    tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
    tgSpringCableActuatorSensorInfo* mySCASensorInfo =
      new tgSpringCableActuatorSensorInfo();
    // Attach the sensor infos to the data logger
    myDataLogger->addSensorInfo(myRodSensorInfo);
    myDataLogger->addSensorInfo(mySCASensorInfo);
    // Next, attach it to the simulation
    //simulation.addDataManager(myDataLogger);
    // and everything else should happen automatically.
    
    // Run until the user stops
    simulation.run();

    //Teardown is handled by delete, so that should be automatic
    return 0;
}
