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
 * @file AppFlemonsSpineContact.cpp
 * @brief Contains the definition function main() for the Flemons Spine Contact
 * application.
 * @author Brian Mirletz
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

// This application
#include "FlemonsSpineModelContact.h"
#include "dev/CPG_feedback/SpineFeedbackControl.h"

// This library
#include "core/tgModel.h"
#include "core/tgSimView.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "core/terrain/tgHillyGround.h"
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
    std::cout << "AppFlemonsSpineContact" << std::endl;

    // First create the world
    const tgWorld::Config config(981); // gravity, cm/sec^2
#if (1)
	btVector3 eulerAngles = btVector3(0.0, 0.0, 0.0);
   btScalar friction = 0.5;
   btScalar restitution = 0.0;
   btVector3 size = btVector3(500.0, 1.5, 500.0);
   btVector3 origin = btVector3(0.0, 0.0, 0.0);
   size_t nx = 50;
   size_t ny = 50;
   double margin = 0.5;
   double triangleSize = 7.5;
   double waveHeight = 5.0;
   double offset = 0.0;
	tgHillyGround::Config groundConfig(eulerAngles, friction, restitution,
									size, origin, nx, ny, margin, triangleSize,
									waveHeight, offset);
	
	tgHillyGround* ground = new tgHillyGround(groundConfig);
	
   tgWorld world(config, ground); 
#else
    tgWorld world(config); 
#endif

    // Second create the view
    const double stepSize = 1.0/1000.0; // Seconds
    const double renderRate = 1.0/60.0; // Seconds
    tgSimView view(world, stepSize, renderRate);

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    const int segments = 12;
    FlemonsSpineModelContact* myModel =
      new FlemonsSpineModelContact(segments);

    /* Required for setting up learning file input/output. */
    const std::string suffix((argc > 1) ? argv[1] : "default");
    
    const int segmentSpan = 3;
    const int numMuscles = 8;
    const int numParams = 2;
    const int segNumber = 6; // For learning results
    const double controlTime = .01;
    const double lowPhase = -1 * M_PI;
    const double highPhase = M_PI;
    const double lowAmplitude = -10 *  30.0;
    const double highAmplitude = 10 * 30.0;
    const double kt = 0.0;
    const double kp = 1000.0;
    const double kv = 200.0;
    const bool def = true;
        
    // Overridden by def being true
    const double cl = 10.0;
    const double lf = -30.0;
    const double hf = 30.0;


    SpineFeedbackControl::Config control_config(segmentSpan, 
                                                numMuscles,
                                                numMuscles,
                                                numParams, 
                                                segNumber, 
                                                controlTime,
                                                lowAmplitude,
                                                highAmplitude,
                                                lowPhase,
                                                highPhase,
                                                kt,
                                                kp,
                                                kv,
                                                def,
                                                cl,
                                                lf,
                                                hf
                                                );
    SpineFeedbackControl* const myControl =
      new SpineFeedbackControl(control_config, suffix, "bmirletz/TetrahedralComplex_Contact/");
    myModel->attach(myControl);
    
    simulation.addModel(myModel);
    
    int i = 0;
    while (i < 3000)
    {
        simulation.run(30000);
    	#ifdef BT_USE_DOUBLE_PRECISION
		std::cout << "Double precision" << std::endl;
	#else
		std::cout << "Single Precision" << std::endl;
	#endif
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
