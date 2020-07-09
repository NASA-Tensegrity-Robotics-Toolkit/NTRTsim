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
 * @file AppQuadControlSpiral.cpp
 * @brief Using Brian's existing spine controller for a quadruped, using passive model with simpler actuation
 * @author Brandon Gigous, Dawn Hustig-Schultz, Brian Mirletz
 * @date August 2016
 * @version 1.0.0
 * $Id$
 */

#include "AppQuadSimpleActuation.h"
#include "dev/btietz/JSONTests/tgCPGJSONLogger.h"
#include "helpers/FileHelpers.h"
#include <json/json.h>

AppQuadSimpleActuation::AppQuadSimpleActuation(int argc, char** argv)
{
    bSetup = false;
    use_graphics = false;
    add_controller = true;
    add_blocks = false;
    add_hills = false;
    all_terrain = false;
    timestep_physics = 1.0f/1000.0f;
    timestep_graphics = 1.0f/60.0f;
    nEpisodes = 1;
    nSteps = 60000;
    nSegments = 7;
    nTypes = 3;

    startX = 0;
    startY = 40; //May need adjustment
    startZ = 0;
    startAngle = 0;
    
    suffix = "default";
    lowerPath = "default";

    handleOptions(argc, argv);
}

bool AppQuadSimpleActuation::setup()
{
    // First create the world
    world = createWorld();

    // Second create the view
    if (use_graphics)
        view = createGraphicsView(world); // For visual experimenting on one tensegrity
    else
        view = createView(world);         // For running multiple episodes

    // Third create the simulation
    simulation = new tgSimulation(*view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    /// @todo add position and angle to configuration
        //FlemonsSpineModelContact* myModel =
      //new FlemonsSpineModelContact(nSegments); 

    //Parameters for the structure:
    const int segments = 7;
    const int hips = 4;
    const int legs = 4;
    const int feet = 4; 

    BigPuppySymmetricSpiral2* myModel = new BigPuppySymmetricSpiral2(segments, hips, legs, feet);

    // Fifth create the controllers, attach to model
    if (add_controller)
    {
		Json::Value root;
		Json::Reader reader;

		std::string resourcePath = "bgigous/AppQuadSimpleActuation/";
		std::string controlFilePath = FileHelpers::getResourcePath(resourcePath);
		std::string controlFilename = controlFilePath + suffix;

        bool parsingSuccessful = reader.parse( FileHelpers::getFileString(controlFilename.c_str()), root );
        if ( !parsingSuccessful )
        {
            // report to the user the failure and their locations in the document.
            std::cout << "Failed to parse configuration\n"
                << reader.getFormattedErrorMessages();
            throw std::invalid_argument("Bad filename for JSON, check that resource path exists");
        }
        // Get the value of the member of root named 'encoding', return 'UTF-8' if there is no
        // such member.
        Json::Value impedenceVals = root.get("impedenceVals", "UTF-8");
        impedenceVals = impedenceVals.get("params", "UTF-8");
        
        // Keep drilling if necessary
        if (impedenceVals[0].isArray())
        {
            impedenceVals = impedenceVals[0];
		}

		const double impedanceMax = 2000.0;

        const int segmentSpan = 3; //Not sure what this will be for mine!
        const int numMuscles = 16; //This may be ok, but confirm. 
        const int numParams = 2;
        const int segNumber = 0; // For learning results
        const double controlTime = .01;
        const double lowPhase = -1*M_PI;
        const double highPhase = M_PI;
        const double lowAmplitude = 0.0;
        const double highAmplitude = 300.0;
        //const double kt = 0.0; //May need to retune kt, kp, and kv
        //const double kp = 2000.0;
        //const double kv = 200.0;
		int j = 0;
		const double kt = impedanceMax * (impedenceVals.get(j, 0.0)).asDouble();
        const double kp = impedanceMax * (impedenceVals.get(1, 0.0)).asDouble();
		const double kv = impedanceMax * (impedenceVals.get(2, 0.0)).asDouble();
        const bool def = false;
            
        // Overridden by def being true
        const double cl = 10.0;
        const double lf = 0.0;
        const double hf = 1.5;
        
        // Feedback parameters... may need to retune
        const double ffMin = 0.0;
        const double ffMax = 1;
        const double afMin = 0.0;
        const double afMax = 200; //200
        const double pfMin = -0.5;//-0.5;
        const double pfMax = 6.28; //6.28;

	const double maxH = 300.0;
	const double minH = 1.0;

        JSONQuadFeedbackControl::Config control_config(segmentSpan, 
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
                                                    hf,
                                                    ffMin,
                                                    ffMax,
                                                    afMin,
                                                    afMax,
                                                    pfMin,
                                                    pfMax,
						    maxH,
						    minH);
        /// @todo fix memory leak that occurs here
       myControl =
        new JSONQuadFeedbackControl(control_config, suffix, lowerPath);

#if (0)        
            tgCPGJSONLogger* const myLogger = 
      new tgCPGJSONLogger("logs/CPGValues.txt");
    
    myControl->attach(myLogger);
#endif        
        myModel->attach(myControl);
    }

    // Sixth add model & controller to simulation
    simulation->addModel(myModel);
    
    if (add_blocks)
    {
        tgModel* blockField = getBlocks();
        simulation->addObstacle(blockField);
    }
    
    bSetup = true;
    return bSetup;
}

void AppQuadSimpleActuation::handleOptions(int argc, char **argv)
{
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("graphics,G", po::value<bool>(&use_graphics), "Test using graphical view")
        ("controller,c", po::value<bool>(&add_controller), "Attach the controller to the model.")
        ("blocks,b", po::value<bool>(&add_blocks)->implicit_value(false), "Add a block field as obstacles.")
        ("hills,H", po::value<bool>(&add_hills)->implicit_value(false), "Use hilly terrain.")
        ("all_terrain,A", po::value<bool>(&all_terrain)->implicit_value(false), "Alternate through terrain types. Only works with graphics off")
        ("phys_time,p", po::value<double>(), "Physics timestep value (Hz). Default=1000")
        ("graph_time,g", po::value<double>(), "Graphics timestep value a.k.a. render rate (Hz). Default = 60")
        ("episodes,e", po::value<int>(&nEpisodes), "Number of episodes to run. Default=1")
        ("steps,s", po::value<int>(&nSteps), "Number of steps per episode to run. Default=60K (60 seconds)")
        ("segments,S", po::value<int>(&nSegments), "Number of segments in the tensegrity spine. Default=6")
        ("start_x,x", po::value<double>(&startX), "X Coordinate of starting position for robot. Default = 0")
        ("start_y,y", po::value<double>(&startY), "Y Coordinate of starting position for robot. Default = 20")
        ("start_z,z", po::value<double>(&startZ), "Z Coordinate of starting position for robot. Default = 0")
        ("angle,a", po::value<double>(&startAngle), "Angle of starting rotation for robot. Degrees. Default = 0")
        ("goal_angle,B", po::value<double>(&goalAngle), "Angle of starting rotation for goal box. Degrees. Default = 0")
        ("learning_controller,l", po::value<std::string>(&suffix), "Which learned controller to write to or use. Default = default")
	("lower_path,P", po::value<std::string>(&lowerPath), "Which resources folder in which you want to store controllers. Default = default")
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

const tgHillyGround::Config AppQuadSimpleActuation::getHillyConfig()
{
    btVector3 eulerAngles = btVector3(0.0, 0.0, 0.0);
    btScalar friction = 0.5;
    btScalar restitution = 0.0;
    // Size doesn't affect hilly terrain
    btVector3 size = btVector3(0.0, 0.1, 0.0);
    btVector3 origin = btVector3(0.0, 0.0, 0.0);
    size_t nx = 100;
    size_t ny = 100;
    double margin = 0.5;
    double triangleSize = 4.0;
    double waveHeight = 2.0;
    double offset = 0.0;
    const tgHillyGround::Config hillGroundConfig(eulerAngles, friction, restitution,
                                    size, origin, nx, ny, margin, triangleSize,
                                    waveHeight, offset);
    return hillGroundConfig;
}

const tgBoxGround::Config AppQuadSimpleActuation::getBoxConfig()
{
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const double friction = 0.5;
    const double restitution = 0.0;
    const btVector3 size(1000.0, 1.5, 1000.0);
    
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll),
                                            friction,
                                            restitution,
                                            size    );
    
    return groundConfig;
}

tgModel* AppQuadSimpleActuation::getBlocks()
{
    // Room to add a config
    tgBlockField* myObstacle = new tgBlockField();
    return myObstacle;
}

tgWorld* AppQuadSimpleActuation::createWorld()
{
    const tgWorld::Config config(
        981 // gravity, cm/sec^2
    );
    
    tgBulletGround* ground;
    
    if (add_hills)
    {
        const tgHillyGround::Config hillGroundConfig = getHillyConfig();
        ground = new tgHillyGround(hillGroundConfig);
    }
    else
    {
        const tgBoxGround::Config groundConfig = getBoxConfig();
        ground = new tgBoxGround(groundConfig);
    }
    
    return new tgWorld(config, ground);
}

tgSimViewGraphics *AppQuadSimpleActuation::createGraphicsView(tgWorld *world)
{
    return new tgSimViewGraphics(*world, timestep_physics, timestep_graphics);
}

tgSimView *AppQuadSimpleActuation::createView(tgWorld *world)
{
    return new tgSimView(*world, timestep_physics, timestep_graphics);
}

bool AppQuadSimpleActuation::run()
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
    
    ///@todo consider app.cleanup()
   delete simulation;
   delete view;
   delete world;
   delete myControl;
    
    return true;
}

void AppQuadSimpleActuation::simulate(tgSimulation *simulation)
{
    for (int i=0; i<nEpisodes; i++) {
        fprintf(stderr,"Episode %d\n", i);
        try
        {
            simulation->run(nSteps);
        }
        catch (std::runtime_error e)
        {
            // Nothing to do here, score will be set to -1
        }
        
        // Don't change the terrain before the last episode to avoid leaks
        if (all_terrain && i != nEpisodes - 1)
        {   
            // Next run has Hills
            if (i % nTypes == 0)
            {
                
                const tgHillyGround::Config hillGroundConfig = getHillyConfig();
                tgBulletGround* ground = new tgHillyGround(hillGroundConfig);
                simulation->reset(ground);
            }
            // Flat
            else if (i % nTypes == 1)
            {
                const tgBoxGround::Config groundConfig = getBoxConfig();
                tgBulletGround* ground = new tgBoxGround(groundConfig);
                simulation->reset(ground);
            }
            // Flat with blocks
            else if (i % nTypes == 2)
            {
                simulation->reset();
                tgModel* obstacle = getBlocks();
                simulation->addObstacle(obstacle);
            }
        }
        // Avoid resetting twice on the last run
        else if (i != nEpisodes - 1)
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
    std::cout << "AppQuadSimpleActuation" << std::endl;
    AppQuadSimpleActuation app (argc, argv);

    if (app.setup())
        app.run();
    
    //Teardown is handled by delete, so that should be automatic
    return 0;
}


