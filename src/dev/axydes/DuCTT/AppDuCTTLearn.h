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

#ifndef APP_DUCTT
#define APP_DUCTT

/**
 * @file AppDuCTTLearn.cpp
 * @brief Contains the definition function main() for the DuCTT app
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

//robot
#include "robot/DuCTTRobotModel.h"

//ducts
#include "ducts/DuctCrossModel.h"
#include "ducts/DuctLModel.h"
#include "ducts/DuctStraightModel.h"
#include "ducts/DuctTeeModel.h"

//controllers
#include "controllers/DuCTTLearningController.h"

// This library
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"

// Boost
#include <boost/program_options.hpp>

// The C++ Standard Library
#include <iostream>

namespace po = boost::program_options;

class AppDuCTTLearn
{
public:
    AppDuCTTLearn(int argc, char** argv);

    /** Setup the simulation */
    bool setup();
    /** Run the simulation */
    bool run();

private:
    /** Parse command line options */
    void handleOptions(int argc, char** argv);

    /** Create the tgWorld object */
    tgWorld *createWorld();

    /** Use for displaying tensegrities in simulation */
    tgSimViewGraphics *createGraphicsView(tgWorld *world);

    /** Use for trial episodes of many tensegrities in an experiment */
    tgSimView *createView(tgWorld *world);

    /** Run a series of episodes for nSteps each */
    void simulate(tgSimulation *simulation);

    tgSimulation* simulation;

    bool use_graphics;
    bool add_controller;
    bool add_duct;
    bool use_manual_params;
    bool use_neuro;

    double timestep_physics; //Seconds
    double timestep_graphics; // Seconds, AKA render rate. Leave at 1/60 for real-time viewing
    int nEpisodes; // Number of episodes ("trial runs")
    int nSteps; // Number of steps in each episode, 60k is 100 seconds (timestep_physics*nSteps)

    std::string paramFile;
    std::string resource_path;
    std::string suffix;

    double startX;
    double startY;
    double startZ;
    double startRotX;
    double startRotY;
    double startRotZ;
    double startAngle;

    int ductAxis;

    bool bSetup;
    bool debug;
};

#endif  // APP_DUCTT
