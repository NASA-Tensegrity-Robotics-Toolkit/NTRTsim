/*
 * Copyright © 2017, United States Government, as represented by the
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
 * @file AppSUPERballChain.cpp
 * @brief Contains the definition function main() for  
 * AppSUPERballChain which builds a YAML SUPERball 2.0.
 * @author Marc Leroy
 * $Id$
 */

// This application
#include "yamlbuilder/TensegrityModel.h"

#include "LengthControllerYAML.h"
#include "HopfControllerML.h"
#include "PhaseOscController.h"

// This library
#include "core/terrain/tgBoxGround.h"
#include "core/terrain/tgHillyGround.h"
#include "core/tgModel.h"
#include "core/tgSimulation.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgWorld.h"
#include "sensors/tgDataLogger2.h"
#include "sensors/tgRodSensorInfo.h"
#include "sensors/tgSpringCableActuatorSensorInfo.h"

// Bullet Physics
#include "LinearMath/btVector3.h"

// The C++ Standard Library
#include <iostream>
#include <string>
#include <vector>

#define NOSCILLATORS 4
#define NSTATES 8
#define USEGRAPHICS 1
#define LOGDATA 0
#define USEHOPFCTLR 0
#define USELENGTHCTLR 0
#define USEPHASEOSCCTLR 1

// Function prototypes
tgBoxGround *createGround();
tgHillyGround *createHillyGround();
tgWorld *createWorld();
tgSimViewGraphics *createGraphicsView(tgWorld *world);
tgSimView *createView(tgWorld *world);
void simulate(tgSimulation *simulation, HopfControllerML* myController);
void simulate(tgSimulation *simulation, PhaseOscController* myController);
void simulate(tgSimulation *simulation, LengthControllerYAML* myController);
std::vector<std::string> selectControlledStrings(std::vector<std::string> tagsToControl);
 

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
      throw std::invalid_argument("No arguments passed in to the application. You need to specify which YAML file you would like to build.");
    }

    // First create the world
    tgWorld *world = createWorld();

    // Second create the view (Choose between with or without graphics)
    #if(USEGRAPHICS)
        tgSimViewGraphics *view = createGraphicsView(world); // For visual experimenting on one tensegrity
    #else
        tgSimView         *view = createView(world);         // For running multiple episodes
    #endif

    // Third create the simulation
    tgSimulation *simulation = new tgSimulation(*view);

    // Create the models with their controllers and add the models to the simulation
    // This constructor for TensegrityModel takes the 'debugging' flag as the
    // second argument.
    TensegrityModel* const myModel = new TensegrityModel(argv[1],false);

    // Parameters for the LengthControllerYAML are specified in that .h file,
    // repeated here:
    double timePassed = 0.0;
    bool   saveToCSV  = false;

    // User-defined initial conditions, shouldn't matter much as Hopf oscillator
    // defines a stable limit cycle
    double hopfState[NSTATES] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double hopfVel[NSTATES]   = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    
    #if(0)
    {
        std::cout << "\e[1;38mStates: ";
        for(int i=0; i<NSTATES; i++)
            std::cout << hopfState[i] << ", ";
        std::cout << "\e[0m" << std::endl;   

        std::cout << "\e[1;38mVelocities: ";
        for(int i=0; i<NSTATES; i++)
            std::cout << hopfVel[i] << ", ";
        std::cout << "\e[0m" << std::endl;   
    }
    #endif

    // Ranges to be examined by the machine learning library
    double hopfOmegaMin = 2.0;
    double hopfOmegaMax = 4.0;
    double hopfMuMin = 0.0;
    double hopfMuMax = 1.0;
    double couplingUpMin = -1.0;
    double couplingUpMax =  0.0;
    double couplingDownMin = -1.0;
    double couplingDownMax =  0.0;
    double couplingNeMin = -1.0;
    double couplingNeMax =  0.0;
    double couplingSeMin = -1.0;
    double couplingSeMax =  0.0;
    double hopfOffsetEvenMin = -0.1;
    double hopfOffsetEvenMax =  0.1;
    double hopfOffsetOddMin =  -0.1;
    double hopfOffsetOddMax =  0.1;

    int ctr = 0;

    std::vector<std::string> tagsToControl;
    tagsToControl = selectControlledStrings(tagsToControl);

    // Changed from examples as this app uses a YAML model for argv[1]
    const std::string suffix((argc > 2) ? argv[2] : "default");

    // Create the controller
    
    #if(USEHOPFCTLR)
        HopfControllerML::Config control_config(hopfOmegaMin,      hopfOmegaMax,
                                                hopfMuMin,         hopfMuMax,
                                                couplingUpMin,     couplingUpMax,
                                                couplingDownMin,   couplingDownMax,
                                                couplingNeMin,     couplingNeMax,
                                                couplingSeMin,     couplingSeMax,
                                                hopfOffsetEvenMin, hopfOffsetEvenMax,
                                                hopfOffsetOddMin,  hopfOffsetOddMax);

        HopfControllerML* const myController = new HopfControllerML(control_config, tagsToControl, timePassed, 
                                                                    ctr, saveToCSV, 
                                                                    hopfState, hopfVel, //hopfAcc, 
                                                                    suffix, "SUPERballPosition/", "Config.ini");
    #endif

    #if(USELENGTHCTLR)   
        double startTime = 35.0;
        double minLength = 0.7;
        double rate = 1.5; //0.25
        LengthControllerYAML* const myController = new LengthControllerYAML(startTime, minLength, rate, tagsToControl);
    #endif

    #if(USEPHASEOSCCTLR)
        PhaseOscController::Config control_config(hopfOmegaMin,      hopfOmegaMax,
                                                  hopfMuMin,         hopfMuMax,
                                                  couplingUpMin,     couplingUpMax,
                                                  couplingDownMin,   couplingDownMax,
                                                  couplingNeMin,     couplingNeMax,
                                                  couplingSeMin,     couplingSeMax,
                                                  hopfOffsetEvenMin, hopfOffsetEvenMax,
                                                  hopfOffsetOddMin,  hopfOffsetOddMax);
        //std::vector<std::vector<double>> params;
        double params[NOSCILLATORS][NSTATES];
        int k = 2;
        for(int i=0; i<NOSCILLATORS; i++)
        {
            for(int j=0; j<NSTATES; j++)
            {
                //std::cout << "Setting up: " << i << ", " << j << ", " << k << ", " << argv[k] << std::endl;
                params[i][j] = atof(argv[k]);
                k++;
            }
        }
        PhaseOscController* const myController = new PhaseOscController(control_config, tagsToControl, timePassed, 
                                                                        ctr, saveToCSV, 
                                                                        hopfState, hopfVel, 
                                                                        suffix, "SUPERballPosition/", "Config.ini",params);
    #endif

    // Attach the controller to the model
    // This is a controller that interacts with a generic TensegrityModel as
    // built by the TensegrityModel file.
    myModel->attach(myController);

    // Add the model to the world
    //simulation.addModel(myModel);
    simulation->addModel(myModel);

    #if(LOGDATA)
        // Add sensors using the new sensing framework
        // A string prefix for the filename
        std::string log_filename = "~/projects/tg_shared/AppSUPERballChain";
        // The time interval between sensor readings:
        double timeInterval = 0.2;
        // First, create the data manager
        tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename);
        //std::cout << myDataLogger->toString() << std::endl;
        // Then, add the model to the data logger
        myDataLogger->addSenseable(myModel);
        // Create sensor infos for all the types of sensors that the data logger
        // will create.
        tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
        tgSpringCableActuatorSensorInfo* mySCASensorInfo =
          new tgSpringCableActuatorSensorInfo();
        // Attach the sensor infos to the data logger
        myDataLogger->addSensorInfo(myRodSensorInfo);
        myDataLogger->addSensorInfo(mySCASensorInfo);
        // Next, attach it to the simulation
        simulation->addDataManager(myDataLogger);
    #endif

    //simulation->run();
    simulate(simulation,myController);

    delete myController;

    // teardown is handled by delete
    return 0;
}


tgWorld *createWorld() {
    const tgWorld::Config config(98.1); // gravity, dm/sec^2  Use this to adjust length scale of world.

    tgBoxGround* ground = createGround();
    //tgHillyGround* ground = createHillyGround();

    return new tgWorld(config, ground);
}


tgBoxGround *createGround() {
    // Determine the angle of the ground in radians. All 0 is flat
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const btVector3 eulerAngles = btVector3(yaw, pitch, roll);  // Default: (0.0, 0.0, 0.0)
    
    /*
    const double friction = 0.5; // Default: 0.5
    const double restitution = 0.0;  // Default: 0.0
    const btVector3 size = btVector3(10000.0, 2, 10000.0); // Default: (500.0, 1.5, 500.0)
    const btVector3 origin = btVector3(0.0, 0.0, 0.0); // Default: (0.0, 0.0, 0.0)
    const tgBoxGround::Config groundConfig(eulerAngles, friction, restitution,
                                           size, origin);
    */

    // the world will delete this
    const tgBoxGround::Config groundConfig(eulerAngles);

    return new tgBoxGround(groundConfig);
}


tgHillyGround *createHillyGround() {
    // Determine the angle of the ground in radians. All 0 is flat
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const btVector3 eulerAngles = btVector3(yaw, pitch, roll);  // Default: (0.0, 0.0, 0.0)
    
    const double friction = 0.25;
    const double restitution = 0.0;
    const btVector3 size   = btVector3(0.0,0.1,0.0);
    const btVector3 origin = btVector3(0.0,0.0,0.0);
    const size_t nx = 100;
    const size_t ny = 100;
    const double margin = 0.5;
    const double triangleSize = 4.0;
    const double waveHeight = 2.0;
    const double offset = 0.0;

    // the world will delete this
    const tgHillyGround::Config hillyGroundConfig(eulerAngles, friction, restitution, size, origin, nx, ny, margin, triangleSize, waveHeight, offset);

    return new tgHillyGround(hillyGroundConfig);
}


/** Use for displaying tensegrities in simulation */
tgSimViewGraphics *createGraphicsView(tgWorld *world) {
    const double timestep_physics = 0.001; //1.0 / 60.0 / 10.0; // Seconds
    const double timestep_graphics = 1.f /60.f; // Seconds, AKA render rate. Leave at 1/60 for real-time viewing
    return new tgSimViewGraphics(*world, timestep_physics, timestep_graphics); 
}


/** Use for trial episodes of many tensegrities in an experiment */
tgSimView *createView(tgWorld *world) {
    const double timestep_physics = 0.001; //1.0 / 60.0 / 10.0; // Seconds
    const double timestep_graphics = 1.f /60.f; // Seconds, AKA render rate. Leave at 1/60 for real-time viewing
    return new tgSimView(*world, timestep_physics, timestep_graphics); 
}


/** Run a series of episodes for nSteps each */
void simulate(tgSimulation *simulation, HopfControllerML* myController) {
    int nEpisodes = 1;//50;  // Number of episodes ("trial runs")
    int nSteps = 30001; // Number of steps in each episode, 60k is 60 seconds (timestep_physics*nSteps)
    for (int i=1; i<=nEpisodes; i++)
    {
        std::cout << "Running episode " << i << " of " << nEpisodes << " with Hopf oscillator" << std::endl;
        try
        {
            if(i!=1)
            {
                std::cout << "RESET" << std::endl;
                simulation->reset();
                myController->resetTimePassed();
            }

            //std::cout << "Starting new run" << std::endl;
            simulation->run(nSteps);
            //std::cout << "End of run" << std::endl;
        }
        catch(const std::invalid_argument& msg)
        {
            std::cout << "\e[1;31mError occured due to: " << msg.what() << "\e[0m" << std::endl << std::endl;
        }
    }
    simulation->reset(); //so that the last episode may be saved in scores.csv
}

void simulate(tgSimulation *simulation, PhaseOscController* myController) {
    int nEpisodes = 1;  // Number of episodes ("trial runs")
    int nSteps = 120001; // Number of steps in each episode, 60k is 60 seconds (timestep_physics*nSteps)
    for (int i=1; i<=nEpisodes; i++)
    {
        std::cout << "Running episode " << i << " of " << nEpisodes << " with Phase oscillator" << std::endl;
        try
        {
            if(i!=1)
            {
                std::cout << "RESET" << std::endl;
                simulation->reset();
                myController->resetTimePassed();
            }

            //std::cout << "Starting new run" << std::endl;
            simulation->run(nSteps);
            //std::cout << "End of run" << std::endl;
        }
        catch(const std::invalid_argument& msg)
        {
            std::cout << "\e[1;31mError occured due to: " << msg.what() << "\e[0m" << std::endl << std::endl;
        }
    }
    simulation->reset(); //so that the last episode may be saved in scores.csv
}

void simulate(tgSimulation *simulation, LengthControllerYAML* myController) {
    int nEpisodes = 10;  // Number of episodes ("trial runs")
    int nSteps = 30001; // Number of steps in each episode, 60k is 60 seconds (timestep_physics*nSteps)
    for (int i=1; i<=nEpisodes; i++)
    {
        std::cout << "Running episode " << i << " of " << nEpisodes << " with non-CPG" << std::endl;
        try
        {
            if(i!=1)
            {
                std::cout << "RESET" << std::endl;
                simulation->reset();
                myController->resetTimePassed();
            }

            //std::cout << "Starting new run" << std::endl;
            simulation->run(nSteps);
            //std::cout << "End of run" << std::endl;
        }
        catch(const std::invalid_argument& msg)
        {
            std::cout << "\e[1;31mError occured due to: " << msg.what() << "\e[0m" << std::endl << std::endl;
        }
    }
    simulation->reset(); //so that the last episode may be saved in scores.csv
}


/**
 * Defines which strings will be controlled
 * (/!\ order sensitive as will be read by the controller in array format)
 * @param[in] tagsToControl empty string vector
 * @return tagsToControl string vector containing strings to be controlled
 */
std::vector<std::string> selectControlledStrings(std::vector<std::string> tagsToControl)
{
/*
    // This vector determines the order with which the strings will be controlled
    std::vector<std::string> listTriangles;
    listTriangles.push_back("NED");
    listTriangles.push_back("NEP");
    listTriangles.push_back("SED");
    listTriangles.push_back("SEP");
    listTriangles.push_back("SWD");
    listTriangles.push_back("SWP");
    listTriangles.push_back("NWD");
    listTriangles.push_back("NWP");

    for(int i=0;i<8;i++)
    {
        if(listTriangles[i] == "NWP") 
        {
            tagsToControl.push_back("SUPERball_string03");
            tagsToControl.push_back("SUPERball_string18");
            tagsToControl.push_back("SUPERball_string13");
        }
        if(listTriangles[i] == "NWD") 
        {    
            tagsToControl.push_back("SUPERball_string04");
            tagsToControl.push_back("SUPERball_string22");
            tagsToControl.push_back("SUPERball_string15");
        }
        if(listTriangles[i] == "SWP") 
        {
            tagsToControl.push_back("SUPERball_string01");
            tagsToControl.push_back("SUPERball_string17");
            tagsToControl.push_back("SUPERball_string09");
        }
        if(listTriangles[i] == "SWD") 
        {
            tagsToControl.push_back("SUPERball_string02");
            tagsToControl.push_back("SUPERball_string21");
            tagsToControl.push_back("SUPERball_string11");
        }
        if(listTriangles[i] == "SEP") 
        {
            tagsToControl.push_back("SUPERball_string05");
            tagsToControl.push_back("SUPERball_string19");
            tagsToControl.push_back("SUPERball_string10");
        }
        if(listTriangles[i] == "SED") 
        {
            tagsToControl.push_back("SUPERball_string06");
            tagsToControl.push_back("SUPERball_string23");
            tagsToControl.push_back("SUPERball_string12");
        }
        if(listTriangles[i] == "NEP") 
        {
            tagsToControl.push_back("SUPERball_string07");
            tagsToControl.push_back("SUPERball_string20");
            tagsToControl.push_back("SUPERball_string14");
        }
        if(listTriangles[i] == "NED") 
        {
            tagsToControl.push_back("SUPERball_string08");
            tagsToControl.push_back("SUPERball_string24");
            tagsToControl.push_back("SUPERball_string16");  
        }
    }
*/    

    //NWP triangle
    tagsToControl.push_back("SUPERball_string03");
    tagsToControl.push_back("SUPERball_string18");
    tagsToControl.push_back("SUPERball_string13");
    
    //NWD triangle
    tagsToControl.push_back("SUPERball_string04");
    tagsToControl.push_back("SUPERball_string22");
    tagsToControl.push_back("SUPERball_string15");

    //SWP triangle
    tagsToControl.push_back("SUPERball_string01");
    tagsToControl.push_back("SUPERball_string17");
    tagsToControl.push_back("SUPERball_string09");

    //SWD triangle
    tagsToControl.push_back("SUPERball_string02");
    tagsToControl.push_back("SUPERball_string21");
    tagsToControl.push_back("SUPERball_string11");

    //SEP triangle
    tagsToControl.push_back("SUPERball_string05");
    tagsToControl.push_back("SUPERball_string19");
    tagsToControl.push_back("SUPERball_string10");
    
    //SED triangle
    tagsToControl.push_back("SUPERball_string06");
    tagsToControl.push_back("SUPERball_string23");
    tagsToControl.push_back("SUPERball_string12");
    
    //NEP triangle
    tagsToControl.push_back("SUPERball_string07");
    tagsToControl.push_back("SUPERball_string20");
    tagsToControl.push_back("SUPERball_string14");

    //NED triangle
    tagsToControl.push_back("SUPERball_string08");
    tagsToControl.push_back("SUPERball_string24");
    tagsToControl.push_back("SUPERball_string16");


     return tagsToControl;
}