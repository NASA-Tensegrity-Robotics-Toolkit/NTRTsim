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

#ifndef APP_MULTI_TERRAIN
#define APP_MULTI_TERRAIN

/**
 * @file AppMultiTerrain.cpp
 * @brief Contains the definition function main() for the Multiple terrains app
 * @author Brian Mirletz, Alexander Xydes
 * $Id$
 */

//robot
#include "dev/btietz/TCContact/FlemonsSpineModelContact.h"

// controller 
#include "dev/CPG_feedback/SpineFeedbackControl.h"
#include "dev/btietz/kinematicString/KinematicSpineCPGControl.h"

// obstacles
#include "models/obstacles/tgBlockField.h"

// This library
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "core/terrain/tgBoxGround.h"
#include "core/terrain/tgHillyGround.h"

// Boost
#include <boost/program_options.hpp>

// The C++ Standard Library
#include <iostream>
#include <string>

namespace po = boost::program_options;

class AppMultiTerrain
{
public:
    AppMultiTerrain(int argc, char** argv);

    /** Setup the simulation */
    bool setup();
    /** Run the simulation */
    bool run();

private:
    /** Parse command line options */
    void handleOptions(int argc, char** argv);

    const tgHillyGround::Config getHillyConfig();
    
    const tgBoxGround::Config getBoxConfig();
    
    tgModel* getBlocks();
    
    /** Create the tgWorld object */
    tgWorld *createWorld();

    /** Use for displaying tensegrities in simulation */
    tgSimViewGraphics *createGraphicsView(tgWorld *world);

    /** Use for trial episodes of many tensegrities in an experiment */
    tgSimView *createView(tgWorld *world);

    /** Run a series of episodes for nSteps each */
    void simulate(tgSimulation *simulation);
    
    
    // Keep these around for cleanup
    tgWorld* world;
    tgSimView* view;
    tgSimulation* simulation;

    bool use_graphics;
    bool add_controller;
    bool add_blocks;
    bool add_hills;
    bool all_terrain;
    double timestep_physics; //Seconds
    double timestep_graphics; // Seconds, AKA render rate. Leave at 1/60 for real-time viewing
    int nEpisodes; // Number of episodes ("trial runs")
    int nSteps; // Number of steps in each episode, 60k is 100 seconds (timestep_physics*nSteps)
    int nSegments; // Number of segments in the tensegrity spine
    int nTypes; // Number of types of terrain to be used. Currently 3

    double startX;
    double startY;
    double startZ;
    double startAngle;
    
    std::string suffix;
    
    bool bSetup;
};

#endif  // APP_DUCTT
