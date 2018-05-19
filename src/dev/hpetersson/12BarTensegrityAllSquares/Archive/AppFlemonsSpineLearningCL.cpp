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
 * @file AppFlemonsSpineLearningCL.cpp
 * @brief Contains the definition function main() for the Flemons Spine Learning
 * application.
 * @author Brian Tietz
 * $Id$
 */

// This application
#include "FlemonsSpineModelLearningCL.h"
// This library
#include "examples/learningSpines/BaseSpineCPGControl.h"
#include "core/tgModel.h"
#include "core/tgSimView.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// The C++ Standard Library
#include <iostream>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name; argv[1], if supplied, is the
 * suffix for the controller
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "AppNestedStructureTest" << std::endl;

    // First create the world
    const tgWorld::Config config(981); // gravity, cm/sec^2
    tgWorld world(config); 

    // Second create the view
    const double stepSize = 1.0/1000.0; // Seconds
    const double renderRate = 1.0/60.0; // Seconds
    tgSimViewGraphics view(world, stepSize, renderRate);

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    const int segments = 12;
    FlemonsSpineModelLearningCL* myModel =
      new FlemonsSpineModelLearningCL(segments);
    
    /* Required for setting up learning file input/output. */
    const std::string suffix((argc > 1) ? argv[1] : "default");
    
    const int segmentSpan = 3;
    const int numMuscles = 4;
    const int numParams = 2;
    const int segNumber = 0;
    
    const double controlTime = 0.1;
    const double lowPhase = -1 * M_PI;
    const double highPhase = M_PI;
    const double lowAmplitude = 0.0;
    const double highAmplitude = 30.0;
    
    const double tension = 0.0;
    const double kPosition = 400.0;
    const double kVelocity = 40.0; 

    BaseSpineCPGControl::Config control_config(segmentSpan, numMuscles, numMuscles, numParams, segNumber, controlTime,
												lowAmplitude, highAmplitude, lowPhase, highPhase,
												tension, kPosition, kVelocity);
    BaseSpineCPGControl* const myControl =
      new BaseSpineCPGControl(control_config, suffix, "learningSpines/OctahedralComplex/");
    myModel->attach(myControl);
    
    simulation.addModel(myModel);
    
    int i = 0;
    while (i < 20000)
    {
        simulation.run(60000);
        simulation.reset();
        i++;
    }
    
    /// @todo Does the model assume ownership of the controller?
    /** No - a single controller could be attached to multiple subjects
    * However, having this here causes a segfault, since there is a call
    * to onTeardown() when the simulation is deleted
    */
    #if (0)
    delete myControl;
    #endif
    //Teardown is handled by delete, so that should be automatic
    return 0;
}
