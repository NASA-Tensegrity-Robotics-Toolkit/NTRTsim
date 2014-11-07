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
 * @file AppDuCTT.cpp
 * @brief Contains the definition function main() for the DuCTT app
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "AppDuCTT.h"

AppDuCTT::AppDuCTT(int argc, char** argv)
{
    use_graphics = true;
    add_controller = false;
    add_duct = false;
    timestep_physics = 1.0f/60.0f/10.0f;
    timestep_graphics = 1.0f/60.0f;
    nEpisodes = 1;
    nSteps = 60000;

    startX = 0;
    startY = 20;
    startZ = 0;

    handleOptions(argc, argv);
}

bool AppDuCTT::setup()
{
    // First create the world
    tgWorld* world = createWorld();

    // Second create the view
    tgSimView *view;
    if (use_graphics)
        view = createGraphicsView(world); // For visual experimenting on one tensegrity
    else
        view = createView(world);         // For running multiple episodes

    // Third create the simulation
    simulation = new tgSimulation(*view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    DuCTTRobotModel::Config c = DuCTTRobotModel::Config(
                btVector3(startX,startY,startZ));
    DuCTTRobotModel* myRobotModel = new DuCTTRobotModel(c);

    // Fifth create the controllers, attach to model
    if (add_controller)
    {
        DuCTTSineWaves* const pPrismControl =
          new DuCTTSineWaves();
        myRobotModel->attach(pPrismControl);
    }

    // Sixth add model & controller to simulation
    simulation->addModel(myRobotModel);

    // Seventh add duct to simulation
    if (add_duct)
    {
        DuctStraightModel* myDuctModel = new DuctStraightModel();
        simulation->addModel(myDuctModel);
    }

    return true;
}

void AppDuCTT::handleOptions(int argc, char **argv)
{
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("graphics,G", po::value<bool>(&use_graphics), "Test using graphical view")
        ("controller,c", po::value<bool>(&add_controller)->implicit_value(true), "Attach the controller to the model.")
        ("duct,d", po::value<bool>(&add_duct)->implicit_value(true), "Add the duct to the simulation.")
        ("phys_time,p", po::value<double>(), "Physics timestep value (Hz). Default=1000")
        ("graph_time,g", po::value<double>(), "Graphics timestep value a.k.a. render rate (Hz). Default = 60")
        ("episodes,e", po::value<int>(&nEpisodes), "Number of episodes to run. Default=1")
        ("steps,s", po::value<int>(&nSteps), "Number of steps per episode to run. Default=60K (100 seconds)")
        ("start_x,x", po::value<double>(&startX), "X Coordinate of starting position for robot. Default = 0")
        ("start_y,y", po::value<double>(&startY), "Y Coordinate of starting position for robot. Default = 20")
        ("start_z,z", po::value<double>(&startZ), "Z Coordinate of starting position for robot. Default = 0")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        exit(0);
    }

    po::notify(vm);

    if (vm.count("phys_time"))
    {
        timestep_physics = 1/vm["phys_time"].as<double>();
        std::cout << "Physics timestep set to: " << timestep_physics << " seconds.\n";
    }

    if (vm.count("graph_time"))
    {
        timestep_graphics = 1/vm["graph_time"].as<double>();
        std::cout << "Graphics timestep set to: " << timestep_graphics << " seconds.\n";
    }
}

tgWorld* AppDuCTT::createWorld()
{
    const tgWorld::Config config(
        981 // gravity, cm/sec^2
    );

    return new tgWorld(config);
}

/** Use for displaying tensegrities in simulation */
tgSimViewGraphics *AppDuCTT::createGraphicsView(tgWorld *world)
{
    return new tgSimViewGraphics(*world, timestep_physics, timestep_graphics);
}

/** Use for trial episodes of many tensegrities in an experiment */
tgSimView *AppDuCTT::createView(tgWorld *world)
{
    return new tgSimView(*world, timestep_physics, timestep_graphics);
}

bool AppDuCTT::run()
{
    if (use_graphics)
    {
        // Run until the user stops
        simulation->run();
    }
    else
    {
        // or run for a specific number of steps
        simulate(simulation);
    }

    return true;
}

/** Run a series of episodes for nSteps each */
void AppDuCTT::simulate(tgSimulation *simulation)
{
    for (int i=0; i<nEpisodes; i++) {
        fprintf(stderr,"Episode %d\n", i);
        simulation->run(nSteps);
        simulation->reset();
    }
}

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "AppDuCTT" << std::endl;
    AppDuCTT app (argc, argv);

    if (app.setup())
        app.run();

    //Teardown is handled by delete, so that should be automatic
    return 0;
}

