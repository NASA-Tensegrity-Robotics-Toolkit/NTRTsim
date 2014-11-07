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
 * @file AppDuCTT.cpp
 * @brief Contains the definition function main() for the DuCTT app
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

// This application
//models
#include "DuCTTRobotModel.h"
#include "DuctStraightModel.h"

//controllers
#include "DuCTTSineWaves.h"

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

class AppDuCTT
{
public:
    AppDuCTT(int argc, char** argv);

    bool setup();
    bool run();

private:
    void handleOptions(int argc, char** argv);
    tgWorld *createWorld();
    tgSimViewGraphics *createGraphicsView(tgWorld *world);
    tgSimView *createView(tgWorld *world);
    void simulate(tgSimulation *simulation);

    tgSimulation* simulation;

    bool use_graphics;
    bool add_controller;
    bool add_duct;
    double timestep_physics; //Seconds
    double timestep_graphics; // Seconds, AKA render rate. Leave at 1/60 for real-time viewing
    int nEpisodes; // Number of episodes ("trial runs")
    int nSteps; // Number of steps in each episode, 60k is 100 seconds (timestep_physics*nSteps)

    double startX;
    double startY;
    double startZ;
};

#endif  // APP_DUCTT
