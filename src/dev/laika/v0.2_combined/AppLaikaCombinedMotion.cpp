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
 * @file AppLaikaCombinedMotion.cpp
 * @brief Contains the definition function main() for AppLaikaCombinedMotion
 * which builds a horizontal spine structure defined in YAML, with a rotating vertebra
 * @author Drew Sabelhaus
 * $Id$
 */

// This application
#include "yamlbuilder/TensegrityModel.h"
#include "CombinedSpineControllerBending.h"
#include "CombinedSpineControllerRotVertPosition.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgRod.h"
#include "core/tgSimulation.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgWorld.h"
#include "sensors/forceplate/ForcePlateModel.h"
#include "sensors/forceplate/ForcePlateSensor.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
#include "core/tgWorldBulletPhysicsImpl.h" // for hinge hack
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h" // for hinge hack
#include "BulletDynamics/Dynamics/btDynamicsWorld.h" // for hinge hack
// The C++ Standard Library
#include <iostream>
#include <string>
#include <vector>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @param[in] argv argv[1] is the path of the YAML encoded structure
 * @return 0
 */
int main(int argc, char** argv)
{
    // For this YAML parser app, need to check that an argument path was
    // passed in.
    if (argv[1] == NULL)
    {
      throw std::invalid_argument("No arguments passed in to the application. You need to specify which YAML file you wouldd like to build.");
    }
  
    // create the ground and world. Specify ground rotation in radians
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);

    const tgWorld::Config config(98.1); // gravity, dm/sec^2
    tgWorld world(config, ground);

    // create the view
    const double timestep_physics = 0.0001; // seconds
    //const double timestep_physics = 0.001;
    const double timestep_graphics = 1.f/60.f; // seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // create the simulation
    tgSimulation simulation(view);

    // create the models with their controllers and add the models to the simulation
    // This constructor for TensegrityModel takes the 'debugging' flag as the
    // second argument.
    TensegrityModel* const myModel = new TensegrityModel(argv[1],true);

    // Attach a controller to the model, if desired.
    // This is a controller that interacts with a generic TensegrityModel as
    // built by the TensegrityModel file, BUT it only actually works
    // with the specific HorizontalSpine YAML file.
    // @TODO: should this throw an error when attached to a model that
    // wasn't built with the HorizontalSpine YAML file?

    // Parameters for the Horizontal Spine Controller are specified in that .h file,
    // repeated here:
    double startTime = 5.0;
    double minLength = 0.8;
    double rate = 0.25;
    std::vector<std::string> tagsToControl;
    // HF is the right horizontal set
    // HL is the bottom horizontal set maybe?
    // HB is the left horizontal set
    // HR is the top horizontal set.
    // BUT, something is wrong here. Probably Bullet's numerical problems.
    tagsToControl.push_back("HR");
    tagsToControl.push_back("testnone");
    //tagsToControl.push_back("HF");
    //tagsToControl.push_back("HB");
    // Call the constructor for the controller
    //CombinedSpineControllerBending* const controller =
    //  new CombinedSpineControllerBending(startTime, minLength, rate, tagsToControl);
    // Attach the controller to the model. Must happen before running the
    // simulation.
    //myModel->attach(controller);

    // Next, we need to get a reference to the Bullet Physics world.
    // This is for passing in to the CombinedSpineControllerRotVertPosition, so it can
    // create the hinge.
    // TO-DO: does this reference get destroyed and re-created?? this will break...
    tgWorld simWorld = simulation.getWorld();
    tgWorldImpl& impl = world.implementation();
    tgWorldBulletPhysicsImpl& bulletWorld = static_cast<tgWorldBulletPhysicsImpl&>(impl);
    btDynamicsWorld* btWorld = &bulletWorld.dynamicsWorld();

    // Create the controller for the rotating vertebra.
    double startTimeRot = 4.0;
    //double startTimeRot = 6.0;
    double setAngle = 0.5; // radians
    // a test: can we do a whole 90 degree rotation?
    //double setAngle = 1.6;
    // hehehe it works but the Laika model explodes.
    std::string rodHingeTag = "rodForHinge";
    CombinedSpineControllerRotVertPosition* rotController =
      new CombinedSpineControllerRotVertPosition( startTimeRot, setAngle,
						  rodHingeTag, btWorld);

    // Add the controller to the YAML model.
    // TO-DO: can we do this after adding the model to the simulation?
    // Will the controller's onSetup method still be
    myModel->attach( rotController );

    // Add the model to the world
    simulation.addModel(myModel);
    
    
    // Finally, run the simulation.
    simulation.run();

    // teardown is handled by delete
    return 0;
}
