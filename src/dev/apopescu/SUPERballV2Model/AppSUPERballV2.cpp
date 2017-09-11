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


// This application
#include "SUPERballV2Model.h"
#include "RestlenPlaybackController.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "sensors/tgDataLogger2.h"
#include "sensors/tgRodSensorInfo.h"
#include "sensors/tgSpringCableActuatorSensorInfo.h"

// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>

/* Library note: The static Bullet libraries libBulletCollision.a, libBulletDynamics.a, libBulletSoftBody.a,
   are linked from the env/lib directory. In contrast, the OpenGL DemoApplication library is linked from 
   env/build/bullet/Demos/OpenGL_FreeGlut directory. These linked libraries are combined into the shared 
   library libcore.so at compile time. */
int main(int argc, char** argv)
{
    // Create the ground and world:
    const double yaw = 0.0;
    const double pitch = 0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    tgBoxGround* ground = new tgBoxGround(groundConfig);
    
    const tgWorld::Config config(98.1); // Set gravity. Use this to adjust length scale of world.
    // Note, by changing this setting to 98.1, we've scaled the world length scale to decimeters.
    
    // Create a tgWorld, which creates a Bullet btDynamicsWorld object.
    tgWorld world(config, ground);

    /* Create the tgSimViewGraphics object, which is a subclass of tgDemoApplication, a Bullet example
       from which many graphics settings are inherited. This initializes the demo application.
       tgSimViewGraphics is also a subclass of tgSimView. */
    const double timestep_physics = 0.001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    /* Create tgSimulation object. Binds the tgSimView to the tgSimulation.
       Then, we set up the tgSimViewGraphics and tgSimView objects, which specifies the debug drawer
       we want to use (draws useful things like coordinate frames, normals, etc.).
       We also create a tgModelVisitor, which is the superclass of the NTRT object rendering 
       implementation, tgBulletRenderer. */
    tgSimulation simulation(view);

    // Create our tensegrity model, which is a subclass of tgModel.
    SUPERballV2Model* const myModel = new SUPERballV2Model();

    // Select the controller to use, and attach it to the model.
    RestlenPlaybackController* const pTC = new RestlenPlaybackController("src/dev/apopescu/SUPERballV2Model/cyl_horiz_pack_len_ten_prestress.csv", 200);
    myModel->attach(pTC);
    
    /* Add our model to the tgSimulation. This calls the setup() method of our tensegrity model.
       So, this creates the tgStructure, tgBuildSpec, and tgStructureInfo, and creates and
       initializes a Bullet btCollisionShape and btRigidBody for each rigid body.
       Finally, we add each rigid body to the btDynamicsWorld (btSoftRigidDynamicsWorld) and add
       as a child to the tgModel. */
    simulation.addModel(myModel);
    
    // Add some data logging
    /*
    std::string log_filename = "~/Desktop/SUPERballTestLog";
    double samplingTimeInterval = 0.05;
    tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename, samplingTimeInterval);
    myDataLogger->addSenseable(myModel);
    
    //tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
    tgSpringCableActuatorSensorInfo* mySCASensorInfo = new tgSpringCableActuatorSensorInfo();
    //myDataLogger->addSensorInfo(myRodSensorInfo);
    myDataLogger->addSensorInfo(mySCASensorInfo);
    simulation.addDataManager(myDataLogger);
    */
    
    /* Run tgSimViewGraphics::run(), which initializes and runs the main loop of the tgDemoApplication.
       This creates the graphics window and runs the display loop. */
    simulation.run();
    
    // Teardown is handled by delete, so that should be automatic
    return 0;
}
