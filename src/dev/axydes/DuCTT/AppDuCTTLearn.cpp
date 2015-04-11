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
 * @file AppDuCTTLearn.cpp
 * @brief Contains the definition function main() for the DuCTT app
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "AppDuCTTLearn.h"

#include "core/tgWorldBulletPhysicsImpl.h"

AppDuCTTLearn::AppDuCTTLearn(int argc, char** argv)
{
    bSetup = false;
    use_graphics = true;
    controller = 0;
    add_duct = false;
    use_manual_params = false;
    use_neuro = false;

    timestep_physics = 1.0f/1000.0f;
    timestep_graphics = 1.0f/60.0f;
    nEpisodes = 1;
    nSteps = 60000;

    startX = 0;
    startY = 1;
    startZ = 0;
    startRotX = 0;
    startRotY = 0;
    startRotZ = 0;
    startAngle = 0;

    ductAxis = 1;
    ductWidth = 33;
    ductHeight = 33;

    paramFile = "";
    resource_path = "axydes/DuCTT/";
    suffix = "DuCTT";

    handleOptions(argc, argv);
}

bool AppDuCTTLearn::setup()
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
                btVector3(startX,startY,startZ),
                btVector3(startRotX,startRotY,startRotZ),
                (startAngle*SIMD_RADS_PER_DEG)
                );
    c.m_storeStringHist = true;
    c.m_debug = debug;
    DuCTTRobotModel* myRobotModel = new DuCTTRobotModel(c);
    tgWorldImpl& impl = world->implementation();
    tgWorldBulletPhysicsImpl& bulletPhysicsImpl = static_cast<tgWorldBulletPhysicsImpl&>(impl);
    myRobotModel->addIgnoredObject((btCollisionObject*)bulletPhysicsImpl.m_pGround);

    // Fifth create the controllers, attach to model
    switch (controller)
    {
    case 1:
        {
            DuCTTLearnCtrl* testLearnCtrl =
                new DuCTTLearnCtrl(5.0, ductAxis, 0.2, use_neuro,
                                            resource_path,
                                            suffix);
            myRobotModel->attach(testLearnCtrl);
        }
        break;
    case 0:
    default:
        {
            DuCTTLearningSines* learningSines =
                new DuCTTLearningSines(5.0, use_manual_params,
                                            paramFile, ductAxis, use_neuro,
                                            resource_path,
                                            suffix);
            myRobotModel->attach(learningSines);
        }
        break;
    }

    // Sixth add model & controller to simulation
    simulation->addModel(myRobotModel);

    // Seventh add duct to simulation
    if (add_duct)
    {
        DuctStraightModel::Config ductConfig;
        ductConfig.m_ductWidth = ductWidth;
        ductConfig.m_ductHeight = ductHeight;
        ductConfig.m_distance = 10000;
        ductConfig.m_axis = ductAxis;
        switch(ductAxis)
        {
        case 0:
            ductConfig.m_startPos = btVector3(-1000,2,0);
            break;
        case 1:
            ductConfig.m_startPos = btVector3(0,-1000,0);
            break;
        case 2:
            ductConfig.m_startPos = btVector3(0,2,-1000);
        default:
            break;
        }

        DuctStraightModel* myDuctModel = new DuctStraightModel(ductConfig);
        simulation->addModel(myDuctModel);
    }

    bSetup = true;
    return bSetup;
}

void AppDuCTTLearn::handleOptions(int argc, char **argv)
{
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("graphics,g", po::value<bool>(&use_graphics), "Test using graphical view")
        ("controller,c", po::value<int>(&controller)->implicit_value(0), "Attach the controller to the model. Default=0=DuCTTLearningSines, 1=DuCTTLearnCtrl")
        ("duct,d", po::value<bool>(&add_duct)->implicit_value(true), "Add the duct to the simulation.")
        ("phys_time,p", po::value<double>(), "Physics timestep value (Hz). Default=600")
        ("graph_time,G", po::value<double>(), "Graphics timestep value a.k.a. render rate (Hz). Default = 60")
        ("episodes,e", po::value<int>(&nEpisodes), "Number of episodes to run. Default=1")
        ("steps,s", po::value<int>(&nSteps), "Number of steps per episode to run. Default=60K (60 seconds)")
        ("start_x,x", po::value<double>(&startX), "X Coordinate of starting position for robot. Default = 0")
        ("start_y,y", po::value<double>(&startY), "Y Coordinate of starting position for robot. Default = 1")
        ("start_z,z", po::value<double>(&startZ), "Z Coordinate of starting position for robot. Default = 0")
        ("rot_x", po::value<double>(&startRotX), "X Coordinate of starting rotation axis for robot. Default = 0")
        ("rot_y", po::value<double>(&startRotY), "Y Coordinate of starting rotation axis for robot. Default = 0")
        //Can only support rotation around the x axis for now.
//        ("rot_z", po::value<double>(&startRotZ), "Z Coordinate of starting rotation axis for robot. Default = 0")
        ("angle,a", po::value<double>(&startAngle), "Angle of starting rotation for robot. Degrees. Default = 0")
        ("paramFile,f", po::value<string>(&paramFile)->implicit_value(""), "File of parameters to use in controller instead of learning the params.")
        ("duct_axis", po::value<int>(&ductAxis)->implicit_value(1), "Axis to extend duct along (X,Y, or Z). Default=Y.")
        ("duct_width,W", po::value<int>(&ductWidth)->implicit_value(33), "Width of the duct (cm)")
        ("duct_height,H", po::value<int>(&ductHeight)->implicit_value(33), "Height of the duct (cm)")
        ("debug,D", po::value<bool>(&debug)->implicit_value(true), "Debug flag, output debug information. Default = false")
        ("neuro,n", po::value<bool>(&use_neuro)->implicit_value(true), "Neuro Evolution flag, use NeuroEvolution instead of AnnealEvolution. Default = false")
        ("resource_path,r", po::value<string>(&resource_path)->implicit_value(resource_path), "Resource path (i.e. axydes/DuCTT). Default = 'axydes/DuCTT'")
        ("suffix", po::value<string>(&suffix)->implicit_value(suffix), "Which learned controller to write to or use. Default = 'DuCTT'")
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

    if (vm.count("paramFile") && vm["paramFile"].as<string>() != "")
    {
        use_manual_params = true;
    }

    if (vm.count("duct_width") && !vm.count("duct_height"))
    {
        ductHeight = vm["duct_width"].as<int>();
    }
    else if (!vm.count("duct_width") && vm.count("duct_height"))
    {
        ductWidth = vm["duct_height"].as<int>();
    }

    if (!resource_path.empty() && (*resource_path.rbegin()) != '/')
    {
        resource_path.append("/");
    }
}

tgWorld* AppDuCTTLearn::createWorld()
{
    const tgWorld::Config config(
        981 // gravity, cm/sec^2
    );

    return new tgWorld(config);
}

tgSimViewGraphics *AppDuCTTLearn::createGraphicsView(tgWorld *world)
{
    return new tgSimViewGraphics(*world, timestep_physics, timestep_graphics);
}

tgSimView *AppDuCTTLearn::createView(tgWorld *world)
{
    return new tgSimView(*world, timestep_physics, timestep_graphics);
}

bool AppDuCTTLearn::run()
{
    if (!bSetup)
    {
        setup();
    }

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

void AppDuCTTLearn::simulate(tgSimulation *simulation)
{
    for (int i=0; i<nEpisodes; i++)
    {
        fprintf(stderr,"Episode %d\n", i);
        try
        {
            simulation->run(nSteps);
            simulation->reset();
        }
        catch (std::runtime_error e)
        {
            simulation->reset();
        }
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
    std::cout << "AppDuCTTLearn" << std::endl;
    AppDuCTTLearn app (argc, argv);

    if (app.setup())
        app.run();

    //Teardown is handled by delete, so that should be automatic
    return 0;
}

