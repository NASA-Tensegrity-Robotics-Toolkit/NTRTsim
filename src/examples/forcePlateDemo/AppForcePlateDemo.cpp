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
 * @file AppForcePlateDemo.cpp
 * @brief Contains the definition function main() for the Force Plate Demo
 * application.
 * $Id$
 */

// This application
#include "sensors/forceplate/ForcePlateModel.h"
// Any additional tgModels that should also be included
#include "examples/3_prism/PrismModel.h"
// The sensor for the force plate
#include "sensors/forceplate/ForcePlateSensor.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "core/tgTags.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <string>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "AppForcePlateDemo" << std::endl;

    // First create the world
    
    const tgWorld::Config config(98.1); // gravity, cm/sec^2  Use this to adjust length scale of world.
    // Note, by changing the setting below from 981 to 98.1, we've
    // scaled the world length scale to decimeters not cm.

    //tgWorld world(config, ground);
    tgWorld world(config);

    // Second create the view
    const double timestep_physics = 0.001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation simulation(view);

    // For this demo, create the force plate by passing in its config struct.
    // There is no need to create another model file here, since the
    // force plate is already a model.
    //
    // Here is the definition of the config struct for ForcePlateModel.
    // See sensors/forceplate/ForcePlateModel.h for more information about
    // each of these parameters.
    
    /**
     * Config( double length = 5.0,
     *	    double width = 5.0,
     *	    double height = 2.0,
     *	    double thickness = 0.1,
     *	    double platethickness = 1.0,
     *	    double wallGap = 0.4, // Used to be 0.2
     *	    double bottomGap = 0.5,
     *	    double massPlate = 10.0, // Worked with > 10?
     *	    double massHousing = 0.0,
     *	    double lateralStiffness = 500.0,
     *	    double verticalStiffness = 1000.0,
     *	    double lateralDamping = 50.0,
     *	    double verticalDamping = 100.0,
     *	    double lateralRestLength = 0.2,
     *	    double verticalRestLength = 0.5,
     *	    double springAnchorOffset = 0.2); // Used to be 0.1
     */

    // NOTE that the elements in the struct itself have DIFFERENT names,
    // the shorter ones, that are described in ForcePlateModel.h.
    // The variable names above are "dummy" names used to pass values in,
    // not hold the values inside the struct itself.

    // We do not need to pass in every one of these parameters,
    // just the ones we want to change from the defaults.
    double length = 20.0;
    double width = 20.0;
    double lateralStiffness = 1500.0;
    double verticalStiffness = 3000.0;
    // NOTE that as with the other actuators, the Unidirectional Compression Spring
    // Actuator inside the ForcePlateModel does not work well when the damping
    // constant is greater than 1/10 the spring constant.
    // For very high stiffnesses, damping must be even less, closer to 1/30.
    // If damping is too large, the plate will explode downward to -infinity.
    double lateralDamping = 50.0;
    double verticalDamping = 100.0;
    
    ForcePlateModel::Config forcePlateConfig(length, width);
    // One way to change the parameters inside a struct is to do so
    // after the struct has been created. This is useful when you want
    // to change a value that's further down in the list of parameters
    // for the struct's constructor, but do not want to change the defaults
    // in between. For example:
    forcePlateConfig.latK = lateralStiffness;
    forcePlateConfig.vertK = verticalStiffness;
    forcePlateConfig.latD = lateralDamping;
    forcePlateConfig.vertD = verticalDamping;
    
    // This line determines the location of the force plate's base.
    // Note that placing it at (0,0,0) might create it below the plane of the
    // world, so something like (0, 2, 0) or (0, 3, 0) might be more appropriate.
    btVector3 forcePlateLocation = btVector3(0,3,0);
    // The force plate takes a boolean that turns debugging information on or off.
    // This is optional: the constructor defaults to "off"/"false".
    bool forcePlateDebugging = true;
    // We can also tag the force plate with an arbitrary string.
    // This will be recorded to the correct place in a log file if a sensor is
    // attached to this force plate.
    //tgTags tags = tgTags("FP1");

    // TEMPORARILY: use a string instead. Not sure why tags didn't work.
    std::string label = "FP1";
    
    // Create the force plate model.
    // Note that in order to call the 'attach' method below, to attach the sensor,
    // this pointer has to be a ForcePlateModel pointer not just a tgModel pointer.
    // That's because the attach method is inherited from tgSubject.
    ForcePlateModel* forcePlate = new ForcePlateModel(forcePlateConfig,
						      forcePlateLocation,
						      forcePlateDebugging,
						      label);


    //DEBUGGING
    std::cout << "In the App, this force plate model has label: "
	      << forcePlate->getLabel() << std::endl;
    
    // Optionally, add a sensor for the force plate.
    // The ForcePlateSensor class takes:
    // a path to the directory where the log file should be stored. NOTE that
    //    this needs to end in a forward slash / or else the file will be wrong!
    // the time between samples to record

    // This is the path to the resources/src/forcePlate/forcePlateDemo/logs folder
    // relative to the compiled executable for this demo application.
    // Change it according to the file structure on your own computer, or also if
    // you use the force plate in another file.
    std::string forcePlateLogPath = "../../../resources/src/forcePlate/forcePlateDemo/logs/";
    // A reasonable time between samples is 0.01 seconds.
    double timeBetweenSamples = 0.1;

    // Create the sensor
    ForcePlateSensor* forceSensor = new ForcePlateSensor(forcePlateLogPath,
							 timeBetweenSamples);

    // Attach the sensor to the force plate
    forcePlate->attach(forceSensor);

    // Add our force plate model to the simulation
    simulation.addModel(forcePlate);

    // We can also add in another model to the simulation.
    // This is how the force plate is used: the plate is added,
    // then the actual tensegrity system model is added.
    // NOTE that this other model must be moved above the force plate,
    // otherwise the two models will collide.
    tgModel* const prismModel = new PrismModel();
    // Move the prism model upwards so it falls onto the plate
    //prismModel->move( btVector3(0, 5, 0) );

    // Then, add the model to the simulation.
    simulation.addModel(prismModel);
    
    // Run until the user stops
    simulation.run();

    //Teardown is handled by delete, so that should be automatic
    return 0;
}
