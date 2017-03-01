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
 * @file AppRotatingVertebraSpine.cpp
 * @brief Contains the definition function main() for AppRotatingVertebraSpine
 * which builds a horizontal spine structure defined in YAML, with a rotating vertebra
 * @author Drew Sabelhaus
 * $Id$
 */

// This application
#include "yamlbuilder/TensegrityModel.h"
#include "HorizontalSpineController.h"
#include "RotatingVertebraController.h"
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
    //HorizontalSpineController* const controller =
    //  new HorizontalSpineController(startTime, minLength, rate, tagsToControl);
    // Attach the controller to the model. Must happen before running the
    // simulation.
    //myModel->attach(controller);

    // Next, we need to get a reference to the Bullet Physics world.
    // This is for passing in to the RotatingVertebraController, so it can
    // create the hinge.
    // TO-DO: does this reference get destroyed and re-created?? this will break...
    tgWorld simWorld = simulation.getWorld();
    tgWorldImpl& impl = world.implementation();
    tgWorldBulletPhysicsImpl& bulletWorld = static_cast<tgWorldBulletPhysicsImpl&>(impl);
    btDynamicsWorld* btWorld = &bulletWorld.dynamicsWorld();

    // Create the controller for the rotating vertebra.
    //double startTimeRot = 2.0;
    double startTimeRot = 6.0;
    //btVector3 startTorqueRot = btVector3(-0.065, 0, 0); // was -0.064
    btVector3 startTorqueRot = btVector3(0, 0, 0);
    //double phaseTwoTimeRot = 3.5;
    double phaseTwoTimeRot = 7.5;
    //btVector3 phaseTwoTorqueRot = btVector3(-0.053225, 0, 0); // was -0.054
    btVector3 phaseTwoTorqueRot = btVector3(0, 0, 0);
    std::string rodHingeTag = "rodForHinge";
    RotatingVertebraController* rotController =
      new RotatingVertebraController( startTimeRot, startTorqueRot,
				      phaseTwoTimeRot, phaseTwoTorqueRot,
				      rodHingeTag, btWorld);

    // Add the controller to the YAML model.
    // TO-DO: can we do this after adding the model to the simulation?
    // Will the controller's onSetup method still be called?
    myModel->attach( rotController );

    // Add the model to the world
    simulation.addModel(myModel);
    
    // Next, add a constraint manually. This will be for the hinged, rotating
    // vertebra. TO-DO: made it a tgModel instead of doing this
    // from the app file? Super hacky...
    /*
    std::vector<tgModel*> all_children = myModel->getDescendants();
    // Pick out the tgRods that will be used for the hinge.
    // These should be tagged rodForHinge in the YAML file.
    std::cout << "Getting all the rods for the hinge..." << std::endl;
    std::vector<tgRod*> allRodsCast = tgCast::filter<tgModel, tgRod>(all_children);
    // A different way:
    std::vector<tgRod*> allRods = myModel->find<tgRod>("rodForHinge");
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
      new btHingeConstraint(*rodA_rb, *rodB_rb, btVector3(-10, 0, 0),
			    btVector3(0, 0, 0), btVector3(1, 0, 0),
			    btVector3(1, 0, 0), false);
    // Next, we need to get a reference to the Bullet Physics world.
    tgWorld simWorld = simulation.getWorld();
    tgWorldImpl& impl = world.implementation();
    tgWorldBulletPhysicsImpl& bulletWorld = static_cast<tgWorldBulletPhysicsImpl&>(impl);
    btDynamicsWorld* btWorld = &bulletWorld.dynamicsWorld();
    // Add the hinge constraint to the world.
    btWorld->addConstraint(rotHinge);

    */

        // Create the force plate by passing in its config struct.
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
    double length = 15.0;
    double width = 15.0;
    //double lateralStiffness = 1000.0;
    double lateralStiffness = 3000.0;
    //double verticalStiffness = 2000.0;
    //double verticalStiffness = 3000.0;
    double verticalStiffness = 5000.0;
    // NOTE that as with the other actuators, the Unidirectional Compression Spring
    // Actuator inside the ForcePlateModel does not work well when the damping
    // constant is greater than 1/10 the spring constant.
    // For very high stiffnesses, damping must be even less, closer to 1/30.
    // If damping is too large, the plate will explode downward to -infinity.
    //double lateralDamping = 100.0;
    double lateralDamping = 50.0;
    //double verticalDamping = 100.0;
    //double verticalDamping = 500.0;
    double verticalDamping = 50.0;
    //double verticalDamping = 200.0;
    
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
    // For this ICRA 2017 paper, we need four identical force plates at different
    // locations.
    btVector3 forcePlateLocationRearLeft =   btVector3( 19,  3, 0);
    btVector3 forcePlateLocationRearRight =  btVector3( 19,  3, 16);
    btVector3 forcePlateLocationFrontLeft =  btVector3( -18, 3, 0);
    btVector3 forcePlateLocationFrontRight = btVector3( -18, 3, 16); 
    // The force plate takes a boolean that turns debugging information on or off.
    // This is optional: the constructor defaults to "off"/"false".
    bool forcePlateDebugging = true;
    // We can also tag the force plate with an arbitrary string.
    // This will be recorded to the correct place in a log file if a sensor is
    // attached to this force plate.
    //tgTags tags = tgTags("FP1");

    // TEMPORARILY: use a string instead. Not sure why tags didn't work.
    std::string labelRearLeft = "FP_RearLeft";
    std::string labelRearRight = "FP_RearRight";
    std::string labelFrontLeft = "FP_FrontLeft";
    std::string labelFrontRight = "FP_FrontRight";
    
    // Create the force plate models.
    // Note that in order to call the 'attach' method below, to attach the sensor,
    // this pointer has to be a ForcePlateModel pointer not just a tgModel pointer.
    // That's because the attach method is inherited from tgSubject.

    // Create one for each foot:
    ForcePlateModel* forcePlateRearLeft = new ForcePlateModel(forcePlateConfig,
						      forcePlateLocationRearLeft,
						      forcePlateDebugging,
						      labelRearLeft);
    ForcePlateModel* forcePlateRearRight = new ForcePlateModel(forcePlateConfig,
						      forcePlateLocationRearRight,
						      forcePlateDebugging,
						      labelRearRight);
    ForcePlateModel* forcePlateFrontLeft = new ForcePlateModel(forcePlateConfig,
						      forcePlateLocationFrontLeft,
						      forcePlateDebugging,
						      labelFrontLeft);
    ForcePlateModel* forcePlateFrontRight = new ForcePlateModel(forcePlateConfig,
						      forcePlateLocationFrontRight,
						      forcePlateDebugging,
						      labelFrontRight);

    //DEBUGGING
    //std::cout << "In the App, this force plate model has label: "
    //	      << forcePlate->getLabel() << std::endl;
    
    // Optionally, add a sensor for the force plate.
    // The ForcePlateSensor class takes:
    // a path to the directory where the log file should be stored. NOTE that
    //    this needs to end in a forward slash / or else the file will be wrong!
    // the time between samples to record

    // This is the path to the resources/src/forcePlate/forcePlateDemo/logs folder
    // relative to the compiled executable for this demo application.
    // Change it according to the file structure on your own computer, or also if
    // you use the force plate in another file.
    std::string forcePlateLogPath = "../../../../resources/src/forcePlate/AppRotatingVertebraSpine/logs/";
    // A reasonable time between samples is 0.01 seconds.
    double timeBetweenSamples = 0.01;

    // Create the sensors
    // Rear Left
    ForcePlateSensor* forceSensorRearLeft =
      new ForcePlateSensor(forcePlateLogPath, forcePlateRearLeft->getLabel(),
			   timeBetweenSamples);
    // Rear Right
    ForcePlateSensor* forceSensorRearRight =
      new ForcePlateSensor(forcePlateLogPath, forcePlateRearRight->getLabel(),
			   timeBetweenSamples);
    // Front Left
    ForcePlateSensor* forceSensorFrontLeft =
      new ForcePlateSensor(forcePlateLogPath, forcePlateFrontLeft->getLabel(),
			   timeBetweenSamples);
    // Front Right
    ForcePlateSensor* forceSensorFrontRight =
      new ForcePlateSensor(forcePlateLogPath, forcePlateFrontRight->getLabel(),
			   timeBetweenSamples);
    
    // Attach the sensor to the force plate
    //UNCOMMENT the following line(s) to get log output.
    //forcePlateRearLeft->attach(forceSensorRearLeft);
    //forcePlateRearRight->attach(forceSensorRearRight);
    //forcePlateFrontLeft->attach(forceSensorFrontLeft);
    //forcePlateFrontRight->attach(forceSensorFrontRight);

    // Add our force plate model to the simulation
    simulation.addModel(forcePlateRearLeft);
    simulation.addModel(forcePlateFrontLeft);
    simulation.addModel(forcePlateRearRight);
    simulation.addModel(forcePlateFrontRight);
    
    // Finally, run the simulation.
    simulation.run();

    // teardown is handled by delete
    return 0;
}
