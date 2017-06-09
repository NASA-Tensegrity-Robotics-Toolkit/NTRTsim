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
 * @file HopfControllerML.cpp
 * @brief Implementation of HopfControllerML.
 * @author Marc Leroy
 * $Id$
 */

// This module
#include "HopfControllerML.h"

// This application
#include "yamlbuilder/TensegrityModel.h"

// This library
#include "core/tgBasicActuator.h"
//#include "core/tgKinematicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"
#include "core/tgRod.h"
#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Adapters/AnnealAdapter.h"
#include "learning/Configuration/configuration.h"

//#include "sensors/tgDataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "helpers/FileHelpers.h"
#include <stdexcept>

// Other required libraries
#include <math.h>
#include <fstream>
#include <cstdio>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <map>
#include <deque>

char timebuffer[12];
std::string primerHopfFileName = "_hopf_results";
std::string primerSpeedFileName = "_speed_hopf_results";
std::string primerCableFileName = "_cable_hopf_results";
std::string primerCOMFileName = "_center_mass_results";
static std::string hopfFileName;
static std::string speedFileName;
static std::string cableFileName; 
static std::string cOMFileName;
static std::vector<std::string> fileNames;

const double CONTROLLER_STOP_TIME = 30000.0;
const double HOPF_AMPLIFIER = 0.5;
const double MATH_PI = 3.14159265359;

HopfControllerML::Config::Config(double hOMin, double hOMax, double hMMin, double hMMax, double cUpMin, double cUpMax,
                                 double cDownMin, double cDownMax, double cNeMin, double cNeMax, double cSeMin, double cSeMax,
                                 double hOffEMin, double hOffEMax, double hOffOMin, double hOffOMax) :
  hopfOmegaMin(hOMin),
  hopfOmegaMax(hOMax),
  hopfMuMin(hMMin),
  hopfMuMax(hMMax),
  couplingUpMin(cUpMin),
  couplingUpMax(cUpMax),
  couplingDownMin(cDownMin),
  couplingDownMax(cDownMax),
  couplingNeMin(cNeMin),
  couplingNeMax(cNeMax),
  couplingSeMin(cSeMin),
  couplingSeMax(cSeMax),
  hopfOffsetEvenMin(hOffEMin),
  hopfOffsetEvenMax(hOffEMax),
  hopfOffsetOddMin(hOffOMin),
  hopfOffsetOddMax(hOffOMax)
{
    /*if (hOMin <= 0)
    {
        throw std::invalid_argument("Hopf oscillator frequency is negative.");
    }
    else if (hOMax > 5.0)
    {
        throw std::invalid_argument("Hopf oscillator frequency is too high.");
    }
    else if (hMMin <= 0)
    {
        throw std::invalid_argument("Hopf oscillator intrinsic frequency is negative.");
    }
    else if (hMMax > 5.0)
    {
        throw std::invalid_argument("Hopf oscillator intrinsic frequency is too high.");
    }
    else if (cMin < -2.0)
    {
        throw std::invalid_argument("Hopf oscillator coupling factors are too low.");
    }
    else if (cMax > 2.0)
    {
        throw std::invalid_argument("Hopf oscillator coupling factors are too high.");
    }*/
}


// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
HopfControllerML::HopfControllerML(HopfControllerML::Config config, std::vector<std::string> tagsToControl, double timePassed, 
                                   int ctr, double initRestLengths, bool saveToCSV,
                                   double hopfStateInit[NSTATES], double hopfVelInit[NSTATES], //double hopfAccInit[NSTATES], 
                                   std::string args, std::string resourcePath, std::string configFile) :
  m_config(config),
  m_tagsToControl(tagsToControl),
  m_timePassed(timePassed),
  ctr(ctr),
  initRestLengths(initRestLengths),
  saveToCSV(saveToCSV),
  // rest for learning library
  configFilename(configFile),
  evolution(args, configFile, resourcePath),
  learning(false)
{
  for(int i=0; i<NSTATES; i++)
  {
    //std::cout << "Starting Hopf Oscillators" << std::endl;
    hopfState[i] = hopfStateInit[i];
    hopfVel[i]   = hopfVelInit[i];
    //hopfAcc[i]   = hopfAccInit[i];

    hopfStateFirst[i] = hopfStateInit[i];
    hopfVelFirst[i]   = hopfVelInit[i];
    //hopfAccFirst[i]   = hopfAccInit[i];
  }


  // Setting up the different log files
  hopfFileName  = setupCSVFiles(primerHopfFileName);
  speedFileName = setupCSVFiles(primerSpeedFileName);
  cableFileName = setupCSVFiles(primerCableFileName);
  cOMFileName   = setupCSVFiles(primerCOMFileName);

  fileNames.push_back(hopfFileName);
  fileNames.push_back(speedFileName);
  fileNames.push_back(cableFileName);
  fileNames.push_back(cOMFileName);


  // Setting up things for the learning library
  std::string path;
  if (resourcePath != "")
  {
    path = FileHelpers::getResourcePath(resourcePath);
  }
  else
  {
    path = "";
  }

  configData.readFile(path + configFilename);
  learning = configData.getintvalue("learning");
}


HopfControllerML::~HopfControllerML()
{
  std::cout << "Destroying" << std::endl;
}


// /!\ Copied from the escape app
// So far, only score used for eventual fitness calculation of an Escape Model
// is the maximum distance from the origin reached during that subject's episode
void HopfControllerML::onTeardown(TensegrityModel& subject) 
{
    std::cout << "Tearing down" << std::endl;
    std::vector<double> scores; //scores[0] == displacement, scores[1] == energySpent
    
    double distance = displacement(subject);
    double energySpent = totalEnergySpent(subject);

    //Invariant: For now, scores must be of size 2 (as required by endEpisode())
    scores.push_back(distance);
    scores.push_back(energySpent);
    
    //std::cout << "Tearing down" << std::endl;
    //adapter.endEpisode(scores);

    // If any of subject's dynamic objects need to be freed, this is the place to do so
}


double HopfControllerML::displacement(TensegrityModel& subject) 
{
    std::vector<double> finalPosition = getBallCOM(subject);

    // 'X' and 'Z' are irrelevant. Both variables measure lateral direction
    //assert(finalPosition[0] > 0); //Negative y-value indicates a flaw in the simulator that run (tensegrity went 'underground')

    const double newX = finalPosition[0];
    const double newZ = finalPosition[2];
    const double oldX = initPosition[0];
    const double oldZ = initPosition[2];

    const double distanceMoved = sqrt((newX-oldX) * (newX-oldX) + 
                                      (newZ-oldZ) * (newZ-oldZ));
    return distanceMoved;
}


double HopfControllerML::totalEnergySpent(TensegrityModel& subject)
{
    double totalEnergySpent=0;

    std::vector<tgBasicActuator* > tmpStrings = cablesWithTags;
    for(size_t i=0; i<tmpStrings.size(); i++) {
        tgSpringCableActuator::SpringCableActuatorHistory stringHist = tmpStrings[i]->getHistory();

        for(size_t j=1; j<stringHist.tensionHistory.size(); j++) {
            const double previousTension = stringHist.tensionHistory[j-1];
            const double previousLength = stringHist.restLengths[j-1];
            const double currentLength = stringHist.restLengths[j];
            //TODO: examine this assumption - free spinning motor may require more power         
            double motorSpeed = (currentLength-previousLength);
            //if(motorSpeed > 0) // Vestigial code
              //  motorSpeed = 0;
            const double workDone = previousTension * motorSpeed / 1000000; // division added by ML
            totalEnergySpent += workDone;
        }
    }
    return totalEnergySpent;
}


/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the setter setupOscillators and helper initializeActuators.
 */
void HopfControllerML::onSetup(TensegrityModel& subject)
{
  resetTimePassed();
  for(int i=0; i<NSTATES; i++)
  {
    //std::cout << "Starting Hopf Oscillators again" << std::endl;
    hopfState[i] = hopfStateFirst[i];
    hopfVel[i]   = hopfVelFirst[i];
    //hopfAcc[i]   = hopfAccFirst[i];
    //std::cout << "Initial states: " << i << " " << hopfState[i] << " " << hopfVel[i] << std::endl;
  }

  initPosition = getBallCOM(subject);
  //std::cout << "Initial Position: " << initPosition[0] << " " << initPosition[1] << " " << initPosition[2] << std::endl;
  //std::cout << "Ctr=" << ctr << ", m_timePassed=" << m_timePassed << std::endl; 

  //Initialize the Learning Adapters
  /*adapter.initialize(&evolution,
                      learning,
                      configData);
  
  std::vector<double> state;
  std::cout << &state << std::endl;
  double dt = 0.0;
  array_2D params = scaleActions(adapter.step(dt, state));*/
  array_2D params(boost::extents[4][8]);
    
  // Choose between keeping the learning library or manually setting parameters
  if(1)
  {    
    std::cout << std::endl << "\e[1;34mManually setting values\e[0m" << std::endl; // ATTENTION: range of couplings changed!
    params[0][0] = 3.83366; //2.00;
    params[0][1] = 0.951536; //0.2;
    params[0][2] = -0.326143; //-0.5;
    params[0][3] = -0.7548; //-0.11;
    params[0][4] = -0.692846; //-0.12;
    params[0][5] = -0.301531; //-0.13;
    params[0][6] = -0.0965834; //-0.14;
    params[0][7] = -0.0477222;//-0.15;

    params[1][0] = 3.81622; //2.01;
    params[1][1] = 0.514954; //0.21;
    params[1][2] = -0.301126; //-0.16;
    params[1][3] = -0.456894; //-0.17;
    params[1][4] = -0.288842; //-0.18;
    params[1][5] = -0.802321; //-0.19;
    params[1][6] = 0.005425; //-0.20;
    params[1][7] = -0.012221;//-0.21;
    
    params[2][0] = 3.66452; //2.02;
    params[2][1] = 0.910582; //0.22;
    params[2][2] = -0.211439; //-0.22;
    params[2][3] = -0.992296; //-0.23;
    params[2][4] = -0.201521; //-0.24;
    params[2][5] = -0.135209; //-0.25;
    params[2][6] = 0.03794; //-0.26;
    params[2][7] = -0.0895263;//-0.27;
    
    params[3][0] = 3.4382; //2.03;
    params[3][1] = 0.931574; //0.23;
    params[3][2] = -0.948404; //-0.28;
    params[3][3] = -0.227634; //-0.29;
    params[3][4] = -0.329029; //-0.30;
    params[3][5] = -0.906597; //-0.31;
    params[3][6] = 0.047229; //-0.32;
    params[3][7] = 0.0110046;//-0.33;
  }
  //for (int i=0; i<NOSCILLATORS; i++)
    //std::cout << "\e[1;31mThe following will be sent to setup: " << i << " " << params[i][0] << " " << params[i][1] << " " << params[i][2] << " " <<params[i][3] << " " << params[i][4] << "\e[0m" << std::endl;
    
  /*// This one is square
  for( std::size_t i = 0; i < configData.getintvalue("numberOfControllers"); i++)
  {
      for( std::size_t j = 0; j < configData.getintvalue("numberOfActions"); j++)
      {
        std::string whichOne = "";
        switch(j)
        {
          case 0: whichOne = ", Omega: "; break;
          case 1: whichOne = ", Mu: "; break;
          case 2: whichOne = ", Coupling: "; break;
        }
        std::cout << i << " " << j << whichOne << params[i][j] << std::endl;
      }
      std::cout << std::endl;
  }*/

  setupOscillators(subject, params);

  //std::cout << "Setting up the HopfControllerML controller." << std::endl;
  //      << "Finding cables with tags: " << m_tagsToControl
  //      << std::endl;
  cablesWithTags = {};
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for(it = m_tagsToControl.begin(); it < m_tagsToControl.end(); it++)
  {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }
  std::cout << "Finished setting up the controller." << std::endl;
  //for(std::size_t i=0; i<=23; i++)
    //std::cout << "Rest length " << i << " " << cablesWithTags[i]->getTags() << " " << cablesWithTags[i]->getHistory().restLengths[0] << std::endl;

  //for(int i=0; i<24; i++)
    //std::cout << initialRLArray[i] << std::endl; 
}


/**
 * This method generates the configuration's settings under the required min/max constraints
 */
array_2D HopfControllerML::scaleActions(std::vector< std::vector <double> > actions)
{
    std::size_t numControllers = configData.getintvalue("numberOfControllers");
    std::size_t numActions     = configData.getintvalue("numberOfActions");
    
    assert( actions.size()    == numControllers);
    assert( actions[0].size() == numActions);
    
    array_2D actionsUpd(boost::extents[numControllers][numActions]);
    
    array_2D limits(boost::extents[2][numActions]);
    
    // Check if we need to update limits
    //assert(numActions == 2);
    
    limits[0][0] = m_config.hopfOmegaMin;
    limits[1][0] = m_config.hopfOmegaMax;
    limits[0][1] = m_config.hopfMuMin;
    limits[1][1] = m_config.hopfMuMax;
    limits[0][2] = m_config.couplingUpMin;
    limits[1][2] = m_config.couplingUpMax;
    limits[0][3] = m_config.couplingDownMin;
    limits[1][3] = m_config.couplingDownMax;
    limits[0][4] = m_config.couplingNeMin;
    limits[1][4] = m_config.couplingNeMax;
    limits[0][5] = m_config.couplingSeMin;
    limits[1][5] = m_config.couplingSeMax;
    limits[0][6] = m_config.hopfOffsetEvenMin;
    limits[1][6] = m_config.hopfOffsetEvenMax;
    limits[0][7] = m_config.hopfOffsetOddMin;
    limits[1][7] = m_config.hopfOffsetOddMax;

    // This one is square
    for( std::size_t i = 0; i < numControllers; i++)
    {
        for( std::size_t j = 0; j < numActions; j++)
        {
            actionsUpd[i][j] = ( actions[i][j] *  
                    (limits[1][j] - limits[0][j])) + limits[0][j];
        }
    }
    return actionsUpd;
}


/**
 * This method transfers the output of scaleActions into the subject's controllers setup
 */
void HopfControllerML::setupOscillators(TensegrityModel& subject, array_2D params)
{
  for(int i=0; i<NOSCILLATORS; i++)
  {
    //std::cout << i << " " << params[i][0] << " " << params[i][1] << " " << params[i][2] << " " <<params[i][3] << " " << params[i][4] << " " << params[i][5] << " " << params[i][6] << " " << params[i][7] << std::endl;
    hopfOmega[i]      = params[i][0];
    hopfMu[i]         = params[i][1];
    coupling[i]       = params[i][2];
    couplingDown[i]   = params[i][3];
    couplingNe[i]     = params[i][4];
    couplingSe[i]     = params[i][5];
    hopfOffsetEven[i] = params[i][6];
    hopfOffsetOdd[i]  = params[i][7];
    //std::cout << i << "\e[1;33m " << hopfOmega[i] << " " << hopfMu[i] << " " << coupling[i] << " " << hopfOffsetEven[i] << " " << hopfOffsetOdd[i] << "\e[0m" << std::endl;
    }  
}


/**
 * The initializeActuators method is called in onSetup to put pointers to 
 * specific actuators in the cablesWithTags array, as well as store the initial
 * rest lengths in the initialRL map.
 */
void HopfControllerML::initializeActuators(TensegrityModel& subject, std::string tag) 
{
  //DEBUGGING
  //  std::cout << "Finding cables with the tag: " << tag << std::endl;
  // Pick out the actuators with the specified tag
  std::vector<tgBasicActuator*> foundActuators = subject.find<tgBasicActuator>(tag);
  //std::vector<tgKinematicActuator*> foundActuators = subject.find<tgKinematicActuator>(tag);
  
  //  std::cout << "The following cables were found and will be controlled: " << std::endl;
  //Iterate through array and output strings to command line
  for (std::size_t i = 0; i < foundActuators.size(); i ++) 
  {	
  //    std::cout << foundActuators[i]->getTags() << std::endl;
    // Also, add the rest length of the actuator at this time
    // to the list of all initial rest lengths.
    initialRL[foundActuators[i]->getTags()] = foundActuators[i]->getRestLength();
    initialRLArray.push_back(foundActuators[i]->getRestLength()); //ML
    //DEBUGGING:
  //    std::cout << "Cable rest length at t=0 is " << initialRL[foundActuators[i]->getTags()] << std::endl;
  }
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  cablesWithTags.insert( cablesWithTags.end(), foundActuators.begin(),
			 foundActuators.end() );
}


/**
 * Method which computes current simulation steps
 */
void HopfControllerML::onStep(TensegrityModel& subject, double dt)
{
  m_timePassed += dt;
  
  if(fabs(m_timePassed-25000*dt) < 0.0000001)
  { 
    testSynchHyp();
    std::vector<double> startingCOM = getBallCOM(subject);
    //std::cout << "\e[1;36mPosition at time " << m_timePassed << " is: " << startingCOM[0] << " " << startingCOM[1] << " " << startingCOM[2] << " (Hopf:" << hopfState[0] << "/" << hopfState[1] << "/" << hopfState[2] << "/" << hopfState[3] << "/" << hopfState[4] << "/" << hopfState[5] << "/" << hopfState[6] << "/" << hopfState[7] << ")\e[0m" << std::endl;
    //for(int i=0; i<24; i++)
      //std::cout << cablesWithTags[i]->getTags() << ", rl=" << (cablesWithTags[i]->getRestLength()) << std::endl;  
  }

  if(m_timePassed >= 3000*dt && m_timePassed < CONTROLLER_STOP_TIME*dt)
  {
    /*if(ctr==0)
    {
      initRestLengths=cablesWithTags[0]->getRestLength();

      for(InitialRestLengths::const_iterator it = initialRL.begin(); it != initialRL.end(); ++it)
      {
        std::cout << it->first << " " << it->second << std::endl;
      }
      std::cout << ctr << " " << m_timePassed << std::endl;
      std::cout << m_config.hopfOmegaMin << " " << m_config.hopfOmegaMax << std::endl;
  
      //std::cout << initRestLengths << std::endl;
      //std::cout << "Test length: " << initialRL.at((tgTags)"SUPERball_string21") << std::endl;
    }*/
    ctr++;
    //if(ctr == 4500)
      //std::cout << hopfState[0] << " " << hopfState[1] << std::endl;
    /*for(InitialRestLengths::const_iterator it = initialRL.begin(); it != initialRL.end(); ++it)
    {
      std::cout << it->first << " " << it->second << std::endl;
    }
    std::cout << std::endl;*/

    updateHopfState(dt);

    //HopfSelector was 01010101
    hopfOscillator(subject, dt, m_timePassed, hopfState, hopfVel, 0, 2, initRestLengths,  0,0,0.0);
    hopfOscillator(subject, dt, m_timePassed, hopfState, hopfVel, 3, 5, initRestLengths,  0,0,MATH_PI);
    hopfOscillator(subject, dt, m_timePassed, hopfState, hopfVel, 6, 8, initRestLengths,  1,0,MATH_PI);
    hopfOscillator(subject, dt, m_timePassed, hopfState, hopfVel, 9, 11, initRestLengths, 1,0,0.0);
    hopfOscillator(subject, dt, m_timePassed, hopfState, hopfVel, 12, 14, initRestLengths,2,0,0.0);
    hopfOscillator(subject, dt, m_timePassed, hopfState, hopfVel, 15, 17, initRestLengths,2,0,MATH_PI);
    hopfOscillator(subject, dt, m_timePassed, hopfState, hopfVel, 18, 20, initRestLengths,3,0,MATH_PI);
    hopfOscillator(subject, dt, m_timePassed, hopfState, hopfVel, 21, 23, initRestLengths,3,0,0.0);
    //for(int i=0; i<cablesWithTags.size(); i++)
      //cablesWithTags[i]->moveMotors(dt);
  }

  if(m_timePassed <= (1+CONTROLLER_STOP_TIME)*dt)
  {
    if(saveToCSV)
    {
      std::vector <double> resultCOM = getBallCOM(subject);
      exportHopfCSV(m_timePassed, hopfState, fileNames, resultCOM);
    }
    if(m_timePassed>CONTROLLER_STOP_TIME*dt)
    {
      double distance = displacement(subject);
      std::cout << "\e[1;36mDONE, traveled " << distance << "\e[0m" << std::endl << std::endl << std::endl;      
      
      if(saveToCSV)
      {
        saveHistLastLengths();
        saveHistRestLengths();
        saveHistDamping();
        saveHistLastVelocities();
        saveHistTension();
      }
    }
  }
}


/**
 * This method checks that the scaled actions will generate a synchronized network of coupled oscillators
 */
void HopfControllerML::testSynchHyp()
{
  double a31 = couplingDown[0];//1.1;
  double a41 = couplingSe[0];//0.11;
  double a13 = coupling[1];//0.12;
  double a23 = couplingNe[1];//0.13;
  double a53 = couplingDown[1];//0.14;
  double a63 = couplingSe[1];//1.15;
  double a35 = coupling[2];//0.14;
  double a45 = couplingNe[2];;//0.13;
  double a75 = couplingDown[2];//0.12;
  double a85 = couplingSe[2];//0.11;
  double a57 = coupling[3];//0.1;
  double a67 = couplingNe[3];//1.13;
  double w13 = hopfOmega[0] - hopfOmega[1];//0.1;
  double w35 = hopfOmega[1] - hopfOmega[2];//0.075;
  double w57 = hopfOmega[2] - hopfOmega[3];//0.05;
  

  double sigma =   a13*a35*a57 -   a13*a35*a67 -   a13*a45*a57 -   a23*a35*a57 + 2*a13*a35*a75 +   a31*a35*a57 + a13*a45*a67 +   a23*a35*a67;
  sigma +=         a23*a45*a57 - 2*a13*a35*a85 - 2*a13*a45*a75 - 2*a23*a35*a75 -   a31*a35*a67 -   a31*a45*a57 - a35*a41*a57 -   a23*a45*a67;
  sigma +=       2*a31*a35*a75 -   a31*a53*a57 + 2*a13*a45*a85 + 2*a23*a35*a85 + 2*a23*a45*a75 +   a31*a45*a67 + a35*a41*a67;
  sigma +=         a41*a45*a57 - 2*a31*a35*a85 - 2*a31*a45*a75 +   a31*a53*a67 +   a31*a57*a63 - 2*a35*a41*a75 + a41*a53*a57 - 2*a23*a45*a85 - a41*a45*a67 - a31*a53*a75;
  sigma +=       2*a31*a45*a85 -   a31*a63*a67 + 2*a35*a41*a85 + 2*a41*a45*a75 -   a41*a53*a67 -   a41*a57*a63 + a31*a53*a85 +   a31*a63*a75;
  sigma +=         a41*a53*a75 - 2*a41*a45*a85 +   a41*a63*a67 -   a31*a63*a85 -   a41*a53*a85 -   a41*a63*a75 + a41*a63*a85;
  
  double s1 = w13*(a35*a57-a35*a67-a45*a57+2*a35*a75-a53*a57+a45*a67-2*a35*a85-2*a45*a75+a53*a67+a57*a63-a53*a75+2*a45*a85-a63*a67+a53*a85+a63*a75-a63*a85);
  s1 -= w57*(a53-a63)*(a75-a85);
  s1 -= w35*(a53-a63)*(a57-a67+a75-a85);
  s1 /= sigma;

  double s2 = -w57 * (a75-a85) * (a13-a23+a31-a41);
  s2 -= w13 * (a13-a23) * (a57-a67+a75-a85);
  s2 -= w35 * (a13-a23+a31-a41) * (a57-a67+a75-a85);
  s2 /= sigma;

  double s3 = w57*(a13*a35-a13*a45-a23*a35+a31*a35+a23*a45-a31*a45-a35*a41-a31*a53+a41*a45+a31*a63+a41*a53-a41*a63);
  s3 -= w13 * (a13-a23) * (a35-a45);
  s3 -= w35 * (a35-a45) * (a13-a23+a31-a41);
  s3 /= sigma;
    
  if(fabs(s1)>=1 || fabs(s2)>=1 || fabs(s3)>=1)
  {
    std::cout << "\e[1;32msigma=" << sigma << ", s1=" << s1 << ", s2=" << s2 << ", s3=" << s3 << "\e[0m" << std::endl;
    throw std::invalid_argument("Not going to synchronize.");
  }         
}


/**
 * This method calls the update of the Hopf oscillators for all controllers
 */
void HopfControllerML::updateHopfState(double dt)
{
  double testOutArray[4] = {0,0,0,0};
  //std::cout << "Now, ";
  for(int i=0; i < NOSCILLATORS; i++)
  {
    compNextHopfState(dt,i);
    //std::cout << "Osc" << i << ": " << hopfState[2*i] << " (" << 10*(1+HOPF_AMPLIFIER*cos(hopfState[2*i])) << ") ";
    /*if((m_timePassed > 9.999 && m_timePassed < 10.001) || (m_timePassed > 19.999 && m_timePassed < 20.001))
    {
      perturbateHopf(i);   
    }*/
    testOutArray[i] = hopfState[2*i];   
  }
  //std::cout << "and in the end, " << testOutArray[0] << " " << testOutArray[1] << " " << testOutArray[2] << " " << testOutArray[3] << std::endl;

  if(0)
  {
    FILE *pFile;
    const char* file1Name = "/home/tensegribuntu/projects/tg_shared/testNewHopf.csv";
    pFile = fopen(file1Name,"a");
    if(pFile!=NULL)
    {
      fprintf(pFile, "%f,%f,%f,%f\n",testOutArray[0],testOutArray[1],testOutArray[2],testOutArray[3]);
      fclose(pFile);
    }
  }
}


/**
 * Computes the next iteration of the Hopf oscillator state thanks to the Euler method
 * TODO: the way the couplings are applied should be further investigated
 * @param[in] dt the time step used for the numerical derivation
 * @return void
 */
void HopfControllerML::compNextHopfState(double dt, int selectedOscillator)
{
  // /!\
  // ATTENTION, SIZE OF COUPLINGARRAY AND COUPLEDSTATE ARRAYS HAS CHANGED WRT BACKUP
  // /!\

  switch(selectedOscillator) //TODO: CHECK COUPLINGS (SEE NOTEBOOK)
  {
    case 0:
      couplingArray[0] = 0;
      couplingArray[1] = couplingDown[0]-couplingSe[0];
      coupledState[0] = 0;
      coupledState[1] = 0;
      coupledState[2] = 2;
      break;
    case 1:
      couplingArray[0] = coupling[1]-couplingNe[1];
      couplingArray[1] = couplingDown[1]-couplingSe[1];
      coupledState[0] = 0;
      coupledState[1] = 2;
      coupledState[2] = 4;
      break;
    case 2:
      couplingArray[0] = coupling[2]-couplingNe[2];
      couplingArray[1] = couplingDown[2]-couplingSe[2];
      coupledState[0] = 2;
      coupledState[1] = 4;
      coupledState[2] = 6;
      break;
    case 3:
      couplingArray[0] = coupling[3]-couplingNe[3];
      couplingArray[1] = 0;
      coupledState[0] = 4;
      coupledState[1] = 6;
      coupledState[2] = 6;
      break;
  }
  
  //double r = sqrt(hopfState[2*selectedOscillator]*hopfState[2*selectedOscillator] + hopfState[2*selectedOscillator+1]*hopfState[2*selectedOscillator+1]);

  hopfVel[2*selectedOscillator]   =   hopfOmega[selectedOscillator] 
                                    + couplingArray[0]*sin(hopfState[coupledState[0]]-hopfState[coupledState[1]])   
                                    - couplingArray[1]*sin(hopfState[coupledState[1]]-hopfState[coupledState[2]]);

  hopfVel[2*selectedOscillator+1]   =   hopfOmega[selectedOscillator+1] 
                                    + couplingArray[0]*sin(hopfState[coupledState[0]]-hopfState[coupledState[1]])   
                                    - couplingArray[1]*sin(hopfState[coupledState[1]]-hopfState[coupledState[2]]);

  //double drdt = (hopfState[2*selectedOscillator]*hopfVel[2*selectedOscillator] + hopfState[2*selectedOscillator+1]*hopfVel[2*selectedOscillator+1])/r;

  /*hopfAcc[2*selectedOscillator]   = -2 * r*drdt * hopfState[2*selectedOscillator]   
                                    + (hopfMu[selectedOscillator]-r*r) * hopfVel[2*selectedOscillator]   
                                    - hopfOmega[selectedOscillator] * hopfVel[2*selectedOscillator+1];

  hopfAcc[2*selectedOscillator+1] = -2 * r*drdt * hopfState[2*selectedOscillator+1] 
                                    + (hopfMu[selectedOscillator]-r*r) * hopfVel[2*selectedOscillator+1] 
                                    + hopfOmega[selectedOscillator] * hopfVel[2*selectedOscillator];*/

  hopfState[2*selectedOscillator]   = hopfState[2*selectedOscillator]   + hopfVel[2*selectedOscillator]*dt  ;// + 0.5*hopfAcc[2*selectedOscillator]*dt*dt;
  hopfState[2*selectedOscillator+1] = hopfState[2*selectedOscillator+1] + hopfVel[2*selectedOscillator+1]*dt;// + 0.5*hopfAcc[2*selectedOscillator+1]*dt*dt;
}


/**
 * Method which perturbates a Hopf oscillator by changing its value to a random value
 * (The stable limit cycle nature of the oscillator will make it return to normal behavior)
 */
void HopfControllerML::perturbateHopf(int selectedOscillator)
{
  // 6 is an arbitrary number here as we have a 2D oscillator based on position, speed and acceleration
  //srand(time(NULL));
  double randArray[4] = {0.0,0.0,0.0,0.0};

  /*for(int i=0; i<4; i++)
  {
    randArray[i] = 0.05*((double)rand()/RAND_MAX);
  }*/

  hopfState[2*selectedOscillator]   = randArray[0];
  hopfState[2*selectedOscillator+1] = randArray[1];
  hopfVel[2*selectedOscillator]     = randArray[2];
  hopfVel[2*selectedOscillator+1]   = randArray[3];
  //hopfAcc[2*selectedOscillator]     = randArray[4];
  //hopfAcc[2*selectedOscillator+1]   = randArray[5];

  //std::cout << randArray[0] << " " << randArray[1] << " " << randArray[2] << " " << randArray[3] << " " << randArray[4] << " " << randArray[5] << std::endl;
}


/**
 * Method which sends the next control input to the actuators, based on Hopf oscillators 
 * @param[in] subject the model of the tensegrity structure
 * @param[in] dt the timestep size in the simulation environment
 * @param[in] m_timePassed the total time passed within the simulation environment
 * @param[in] hopfState the state in which the Hopf oscillator is at the according time
 * @param[in] hopfVel the velocity which the Hopf oscillator has at the according time
 * @param[in] hopfAcc the acceleration which the Hopf oscillator has at the according time
 * @param[in] firstCable the first in the list of strings following the same control law
 * @param[in] lastCable the last in the list of strings following the same control law
 * @param[in] initRestLengths the initial rest length of a string
 * @param[in] hopfSelector number defining which component of the oscillator will be used
 * @return void
 */
void HopfControllerML::hopfOscillator(TensegrityModel& subject, double dt, double m_timePassed, double *hopfState, double *hopfVel, //double *hopfAcc,
                                      int firstCable, int lastCable, double initRestLengths, int selectedOscillator, int hopfSelector, double phaseOffset)
{
  for (std::size_t i = firstCable; i <= lastCable; i ++) 
  {  
    double bufferVar = 0;
    double offsetVar = 0;
    
    if(hopfSelector==0)
    {
      bufferVar = hopfState[2*selectedOscillator];
      offsetVar = hopfOffsetEven[2*selectedOscillator];
      //cablesWithTags[i]->setControlInput((HOPF_AMPLIFIER*bufferVar+1+offsetEven)*((cablesWithTags[i]->getHistory()).restLengths[0]), dt);
    }
    else
    {
      bufferVar = hopfState[2*selectedOscillator+1];
      offsetVar = hopfOffsetOdd[2*selectedOscillator+1];
      //cablesWithTags[i]->setControlInput((HOPF_AMPLIFIER*bufferVar+1+offsetOdd)*((cablesWithTags[i]->getHistory()).restLengths[0]), dt);
    }
    
    //std::cout << "Cable " << cablesWithTags[i]->getTags() << ", control: " << ((cablesWithTags[i]->getHistory()).restLengths[0])*(1+HOPF_AMPLIFIER*cos(bufferVar+phaseOffset)) << std::endl;
    cablesWithTags[i]->setControlInput(((cablesWithTags[i]->getHistory()).restLengths[0])*(1+HOPF_AMPLIFIER*cos(bufferVar+phaseOffset)), dt);
  }
}


/**
 * Method which resets the time counter (useful for multi-run simulations)
 */
void HopfControllerML::resetTimePassed()
{
  m_timePassed = 0;
  ctr = 0;
  bool random = false;
  
  for(int i=0; i<NSTATES; i++)
  {
    //std::cout << "States right before reset: " << hopfState[i] << " " << hopfVel[i] << std::endl;
    hopfState[i] = 0.0;
    hopfVel[i] = 0.0; 
    /*if(random)
    {
    /hopfState[i] = (double)(rand()%1000)/500-1;
      hopfVel[i] = (double)(rand()%1000)/500-1;
    }
  }
  if(random)
  {
    std::cout << "\e[1;38mStates: ";
    for(int i=0; i<NSTATES; i++)
        std::cout << hopfState[i] << ", ";
    std::cout << "\e[0m" << std::endl;   

    std::cout << "\e[1;38mVelocities: ";
    for(int i=0; i<NSTATES; i++)
        std::cout << hopfVel[i] << ", ";
    std::cout << "\e[0m" << std::endl;*/   
  }
}


// Return the center of mass of this model
// Pre-condition: This model has 6 rods
std::vector<double> HopfControllerML::getBallCOM(TensegrityModel& subject) 
{   
    //std::vector <tgRod*> rods = find<tgRod>("tgRodInfo");
    //assert(!rods.empty());

    btVector3 ballCenterOfMass(0, 0, 0);

    std::vector<tgRod*> foundRods = subject.find<tgRod>("superball_rod");
  
    double ballMass = 0.0; 
    for (std::size_t i = 0; i < foundRods.size(); i++) {
        const tgRod* const rod = foundRods[i];
        assert(rod != NULL);
        const double rodMass = rod->mass();
        const btVector3 rodCenterOfMass = rod->centerOfMass();
        //std::cout << i << ", " << rodCenterOfMass << ", ";
        ballCenterOfMass += rodCenterOfMass * rodMass;
        ballMass += rodMass;
    }
    //std::cout << std::endl;

    //assert(ballMass > 0.0);
    ballCenterOfMass /= ballMass;

    // Copy to the result std::vector
    std::vector<double> result(3);
    for (size_t i = 0; i < 3; ++i) 
    { 
      result[i] = ballCenterOfMass[i]; 
    }
    //std::cout << "X=" << result[0] << ", Y=" << result[1] << ", Z=" << result[2] << std::endl;
    return result;
}


/**
 * Setup of the timestamped filenames that will be used to export data in .csv format 
 * @param[in] fileDataType string describing start of file's name, e.g. speed or tension
 * @return outputFileName string of full path towards timestamped file
 */
std::string HopfControllerML::setupCSVFiles(std::string fileDataType)
{  
  /*srand(time(NULL));
  time_t rawtime;
  struct tm *timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(timebuffer,12,"%m_%d_%H_%M",timeinfo);
  std::string outputFileName = "/home/tensegribuntu/projects/tg_shared/" + (std::string)timebuffer + fileDataType + ".csv";*/
  std::string outputFileName = "/home/tensegribuntu/projects/tg_shared/__changeLogsBack.csv";
 
  return (std::string)outputFileName;
}


/**
 * Properly saves the required data in the different files generated by the prior setupCSVFiles calls
 * This method is called at each simulation timestep and the new data is thus appended in the files
 * @param[in] t time having passed within the simulator's world
 * @param[in] hopfState current state of the Hopf oscillator
 * @param[in] fileNames name of the files in which data will be stored
 * @return void
 */
void HopfControllerML::exportHopfCSV(double t, double *hopfState, std::vector<std::string> fileNames, std::vector <double> resultCOM)
{
  FILE *pFile;
  const char* file1Name = fileNames[0].c_str();
  pFile = fopen(file1Name,"a");
  if(pFile!=NULL){
    fprintf(pFile, "%f,%f,%f,%f,%f,%f,%f,%f,%f\n",t,hopfState[0],hopfState[1],hopfState[2],hopfState[3],hopfState[4],hopfState[5],hopfState[6],hopfState[7]);
    fclose(pFile);
  }

  FILE *pFile2;
  const char* file2Name = fileNames[1].c_str();
  pFile2 = fopen(file2Name,"a");
  if(pFile2!=NULL){
    fprintf(pFile2, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",t, 
                      cablesWithTags[0]->getVelocity(),
                      cablesWithTags[1]->getVelocity(),
                      cablesWithTags[2]->getVelocity(),
                      cablesWithTags[3]->getVelocity(),
                      cablesWithTags[4]->getVelocity(),
                      cablesWithTags[5]->getVelocity(),
                      cablesWithTags[6]->getVelocity(),
                      cablesWithTags[7]->getVelocity(),
                      cablesWithTags[8]->getVelocity(),
                      cablesWithTags[9]->getVelocity(),
                      cablesWithTags[10]->getVelocity(),
                      cablesWithTags[11]->getVelocity(),
                      cablesWithTags[12]->getVelocity(),
                      cablesWithTags[13]->getVelocity(),
                      cablesWithTags[14]->getVelocity(),
                      cablesWithTags[15]->getVelocity(),
                      cablesWithTags[16]->getVelocity(),
                      cablesWithTags[17]->getVelocity(),
                      cablesWithTags[18]->getVelocity(),
                      cablesWithTags[19]->getVelocity(),
                      cablesWithTags[20]->getVelocity(),
                      cablesWithTags[21]->getVelocity(),
                      cablesWithTags[22]->getVelocity(),
                      cablesWithTags[23]->getVelocity());
    fclose(pFile2);
  }

  double bufferVar = initRestLengths*(1+hopfState[0]);
  FILE *pFile3;
  const char* file3Name = fileNames[2].c_str();
  pFile3 = fopen(file3Name,"a");
  if(pFile3!=NULL){
    fprintf(pFile3, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",t, bufferVar,
                      cablesWithTags[0]->getRestLength(),
                      cablesWithTags[1]->getRestLength(),
                      cablesWithTags[2]->getRestLength(),
                      cablesWithTags[3]->getRestLength(),
                      cablesWithTags[4]->getRestLength(),
                      cablesWithTags[5]->getRestLength(),
                      cablesWithTags[6]->getRestLength(),
                      cablesWithTags[7]->getRestLength(),
                      cablesWithTags[8]->getRestLength(),
                      cablesWithTags[9]->getRestLength(),
                      cablesWithTags[10]->getRestLength(),
                      cablesWithTags[11]->getRestLength(),
                      cablesWithTags[12]->getRestLength(),
                      cablesWithTags[13]->getRestLength(),
                      cablesWithTags[14]->getRestLength(),
                      cablesWithTags[15]->getRestLength(),
                      cablesWithTags[16]->getRestLength(),
                      cablesWithTags[17]->getRestLength(),
                      cablesWithTags[18]->getRestLength(),
                      cablesWithTags[19]->getRestLength(),
                      cablesWithTags[20]->getRestLength(),
                      cablesWithTags[21]->getRestLength(),
                      cablesWithTags[22]->getRestLength(),
                      cablesWithTags[23]->getRestLength());
    fclose(pFile3);
  }

  FILE *pFile4;
  const char* file4Name = fileNames[3].c_str();
  pFile4 = fopen(file4Name,"a");
  if(pFile4!=NULL){
    fprintf(pFile4, "%f,%f,%f,%f\n",t,resultCOM[0],resultCOM[1],resultCOM[2]);
    fclose(pFile4);
  }
}


void HopfControllerML::saveHistLastLengths()
{
  std::deque<double> lastLengthsCable0((cablesWithTags[0]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable1((cablesWithTags[1]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable2((cablesWithTags[2]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable3((cablesWithTags[3]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable4((cablesWithTags[4]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable5((cablesWithTags[5]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable6((cablesWithTags[6]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable7((cablesWithTags[7]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable8((cablesWithTags[8]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable9((cablesWithTags[9]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable10((cablesWithTags[10]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable11((cablesWithTags[11]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable12((cablesWithTags[12]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable13((cablesWithTags[13]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable14((cablesWithTags[14]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable15((cablesWithTags[15]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable16((cablesWithTags[16]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable17((cablesWithTags[17]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable18((cablesWithTags[18]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable19((cablesWithTags[19]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable20((cablesWithTags[20]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable21((cablesWithTags[21]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable22((cablesWithTags[22]->getHistory()).lastLengths);
  std::deque<double> lastLengthsCable23((cablesWithTags[23]->getHistory()).lastLengths);
  
  FILE *pFile;
  std::string fileNameStr = "/home/tensegribuntu/projects/tg_shared/";
  fileNameStr += timebuffer;
  fileNameStr += "_histLogLastLengths.csv";
  const char* fileName = fileNameStr.c_str();
  pFile = fopen(fileName,"a");
  if(pFile!=NULL){
    for(int i=0;i<lastLengthsCable0.size();i++)
    {
      double t = (double)i/1000;
      fprintf(pFile, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",t,
                      lastLengthsCable0[i],
                      lastLengthsCable1[i],
                      lastLengthsCable2[i],
                      lastLengthsCable3[i],
                      lastLengthsCable4[i],
                      lastLengthsCable5[i],
                      lastLengthsCable6[i],
                      lastLengthsCable7[i],
                      lastLengthsCable8[i],
                      lastLengthsCable9[i],
                      lastLengthsCable10[i],
                      lastLengthsCable11[i],
                      lastLengthsCable12[i],
                      lastLengthsCable13[i],
                      lastLengthsCable14[i],
                      lastLengthsCable15[i],
                      lastLengthsCable16[i],
                      lastLengthsCable17[i],
                      lastLengthsCable18[i],
                      lastLengthsCable19[i],
                      lastLengthsCable20[i],
                      lastLengthsCable21[i],
                      lastLengthsCable22[i],
                      lastLengthsCable23[i]);
    }
    fclose(pFile);
  }
}


void HopfControllerML::saveHistRestLengths()
{
  std::deque<double> restLengthsCable0((cablesWithTags[0]->getHistory()).restLengths);
  std::deque<double> restLengthsCable1((cablesWithTags[1]->getHistory()).restLengths);
  std::deque<double> restLengthsCable2((cablesWithTags[2]->getHistory()).restLengths);
  std::deque<double> restLengthsCable3((cablesWithTags[3]->getHistory()).restLengths);
  std::deque<double> restLengthsCable4((cablesWithTags[4]->getHistory()).restLengths);
  std::deque<double> restLengthsCable5((cablesWithTags[5]->getHistory()).restLengths);
  std::deque<double> restLengthsCable6((cablesWithTags[6]->getHistory()).restLengths);
  std::deque<double> restLengthsCable7((cablesWithTags[7]->getHistory()).restLengths);
  std::deque<double> restLengthsCable8((cablesWithTags[8]->getHistory()).restLengths);
  std::deque<double> restLengthsCable9((cablesWithTags[9]->getHistory()).restLengths);
  std::deque<double> restLengthsCable10((cablesWithTags[10]->getHistory()).restLengths);
  std::deque<double> restLengthsCable11((cablesWithTags[11]->getHistory()).restLengths);
  std::deque<double> restLengthsCable12((cablesWithTags[12]->getHistory()).restLengths);
  std::deque<double> restLengthsCable13((cablesWithTags[13]->getHistory()).restLengths);
  std::deque<double> restLengthsCable14((cablesWithTags[14]->getHistory()).restLengths);
  std::deque<double> restLengthsCable15((cablesWithTags[15]->getHistory()).restLengths);
  std::deque<double> restLengthsCable16((cablesWithTags[16]->getHistory()).restLengths);
  std::deque<double> restLengthsCable17((cablesWithTags[17]->getHistory()).restLengths);
  std::deque<double> restLengthsCable18((cablesWithTags[18]->getHistory()).restLengths);
  std::deque<double> restLengthsCable19((cablesWithTags[19]->getHistory()).restLengths);
  std::deque<double> restLengthsCable20((cablesWithTags[20]->getHistory()).restLengths);
  std::deque<double> restLengthsCable21((cablesWithTags[21]->getHistory()).restLengths);
  std::deque<double> restLengthsCable22((cablesWithTags[22]->getHistory()).restLengths);
  std::deque<double> restLengthsCable23((cablesWithTags[23]->getHistory()).restLengths);
  
  FILE *pFile;
  std::string fileNameStr = "/home/tensegribuntu/projects/tg_shared/";
  fileNameStr += timebuffer;
  fileNameStr += "_histLogRestLengths.csv";
  const char* fileName = fileNameStr.c_str();
  pFile = fopen(fileName,"a");
  if(pFile!=NULL){
    for(int i=0;i<restLengthsCable0.size();i++)
    {
      double t = (double)i/1000;
      fprintf(pFile, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",t,
                      restLengthsCable0[i],
                      restLengthsCable1[i],
                      restLengthsCable2[i],
                      restLengthsCable3[i],
                      restLengthsCable4[i],
                      restLengthsCable5[i],
                      restLengthsCable6[i],
                      restLengthsCable7[i],
                      restLengthsCable8[i],
                      restLengthsCable9[i],
                      restLengthsCable10[i],
                      restLengthsCable11[i],
                      restLengthsCable12[i],
                      restLengthsCable13[i],
                      restLengthsCable14[i],
                      restLengthsCable15[i],
                      restLengthsCable16[i],
                      restLengthsCable17[i],
                      restLengthsCable18[i],
                      restLengthsCable19[i],
                      restLengthsCable20[i],
                      restLengthsCable21[i],
                      restLengthsCable22[i],
                      restLengthsCable23[i]);
    }
    fclose(pFile);
  }
}


void HopfControllerML::saveHistDamping()
{
  std::deque<double> dampingCable0((cablesWithTags[0]->getHistory()).dampingHistory);
  std::deque<double> dampingCable1((cablesWithTags[1]->getHistory()).dampingHistory);
  std::deque<double> dampingCable2((cablesWithTags[2]->getHistory()).dampingHistory);
  std::deque<double> dampingCable3((cablesWithTags[3]->getHistory()).dampingHistory);
  std::deque<double> dampingCable4((cablesWithTags[4]->getHistory()).dampingHistory);
  std::deque<double> dampingCable5((cablesWithTags[5]->getHistory()).dampingHistory);
  std::deque<double> dampingCable6((cablesWithTags[6]->getHistory()).dampingHistory);
  std::deque<double> dampingCable7((cablesWithTags[7]->getHistory()).dampingHistory);
  std::deque<double> dampingCable8((cablesWithTags[8]->getHistory()).dampingHistory);
  std::deque<double> dampingCable9((cablesWithTags[9]->getHistory()).dampingHistory);
  std::deque<double> dampingCable10((cablesWithTags[10]->getHistory()).dampingHistory);
  std::deque<double> dampingCable11((cablesWithTags[11]->getHistory()).dampingHistory);
  std::deque<double> dampingCable12((cablesWithTags[12]->getHistory()).dampingHistory);
  std::deque<double> dampingCable13((cablesWithTags[13]->getHistory()).dampingHistory);
  std::deque<double> dampingCable14((cablesWithTags[14]->getHistory()).dampingHistory);
  std::deque<double> dampingCable15((cablesWithTags[15]->getHistory()).dampingHistory);
  std::deque<double> dampingCable16((cablesWithTags[16]->getHistory()).dampingHistory);
  std::deque<double> dampingCable17((cablesWithTags[17]->getHistory()).dampingHistory);
  std::deque<double> dampingCable18((cablesWithTags[18]->getHistory()).dampingHistory);
  std::deque<double> dampingCable19((cablesWithTags[19]->getHistory()).dampingHistory);
  std::deque<double> dampingCable20((cablesWithTags[20]->getHistory()).dampingHistory);
  std::deque<double> dampingCable21((cablesWithTags[21]->getHistory()).dampingHistory);
  std::deque<double> dampingCable22((cablesWithTags[22]->getHistory()).dampingHistory);
  std::deque<double> dampingCable23((cablesWithTags[23]->getHistory()).dampingHistory);
  
  FILE *pFile;
  std::string fileNameStr = "/home/tensegribuntu/projects/tg_shared/";
  fileNameStr += timebuffer;
  fileNameStr += "_histLogDamping.csv";
  const char* fileName = fileNameStr.c_str();
  pFile = fopen(fileName,"a");
  if(pFile!=NULL){
    for(int i=0;i<dampingCable0.size();i++)
    {
      double t = (double)i/1000;
      fprintf(pFile, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",t,
                      dampingCable0[i],
                      dampingCable1[i],
                      dampingCable2[i],
                      dampingCable3[i],
                      dampingCable4[i],
                      dampingCable5[i],
                      dampingCable6[i],
                      dampingCable7[i],
                      dampingCable8[i],
                      dampingCable9[i],
                      dampingCable10[i],
                      dampingCable11[i],
                      dampingCable12[i],
                      dampingCable13[i],
                      dampingCable14[i],
                      dampingCable15[i],
                      dampingCable16[i],
                      dampingCable17[i],
                      dampingCable18[i],
                      dampingCable19[i],
                      dampingCable20[i],
                      dampingCable21[i],
                      dampingCable22[i],
                      dampingCable23[i]);
    }
    fclose(pFile);
  }
}


void HopfControllerML::saveHistLastVelocities()
{
  std::deque<double> lastVelocitiesCable0((cablesWithTags[0]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable1((cablesWithTags[1]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable2((cablesWithTags[2]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable3((cablesWithTags[3]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable4((cablesWithTags[4]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable5((cablesWithTags[5]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable6((cablesWithTags[6]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable7((cablesWithTags[7]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable8((cablesWithTags[8]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable9((cablesWithTags[9]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable10((cablesWithTags[10]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable11((cablesWithTags[11]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable12((cablesWithTags[12]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable13((cablesWithTags[13]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable14((cablesWithTags[14]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable15((cablesWithTags[15]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable16((cablesWithTags[16]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable17((cablesWithTags[17]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable18((cablesWithTags[18]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable19((cablesWithTags[19]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable20((cablesWithTags[20]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable21((cablesWithTags[21]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable22((cablesWithTags[22]->getHistory()).lastVelocities);
  std::deque<double> lastVelocitiesCable23((cablesWithTags[23]->getHistory()).lastVelocities);
  
  FILE *pFile;
  std::string fileNameStr = "/home/tensegribuntu/projects/tg_shared/";
  fileNameStr += timebuffer;
  fileNameStr += "_histLogLastVelocities.csv";
  const char* fileName = fileNameStr.c_str();
  pFile = fopen(fileName,"a");
  if(pFile!=NULL){
    for(int i=0;i<lastVelocitiesCable0.size();i++)
    {
      double t = (double)i/1000;
      fprintf(pFile, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",t,
                      lastVelocitiesCable0[i],
                      lastVelocitiesCable1[i],
                      lastVelocitiesCable2[i],
                      lastVelocitiesCable3[i],
                      lastVelocitiesCable4[i],
                      lastVelocitiesCable5[i],
                      lastVelocitiesCable6[i],
                      lastVelocitiesCable7[i],
                      lastVelocitiesCable8[i],
                      lastVelocitiesCable9[i],
                      lastVelocitiesCable10[i],
                      lastVelocitiesCable11[i],
                      lastVelocitiesCable12[i],
                      lastVelocitiesCable13[i],
                      lastVelocitiesCable14[i],
                      lastVelocitiesCable15[i],
                      lastVelocitiesCable16[i],
                      lastVelocitiesCable17[i],
                      lastVelocitiesCable18[i],
                      lastVelocitiesCable19[i],
                      lastVelocitiesCable20[i],
                      lastVelocitiesCable21[i],
                      lastVelocitiesCable22[i],
                      lastVelocitiesCable23[i]);                      
    }
    fclose(pFile);
  }
}


void HopfControllerML::saveHistTension()
{
  std::deque<double> tensionHistoryCable0((cablesWithTags[0]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable1((cablesWithTags[1]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable2((cablesWithTags[2]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable3((cablesWithTags[3]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable4((cablesWithTags[4]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable5((cablesWithTags[5]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable6((cablesWithTags[6]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable7((cablesWithTags[7]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable8((cablesWithTags[8]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable9((cablesWithTags[9]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable10((cablesWithTags[10]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable11((cablesWithTags[11]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable12((cablesWithTags[12]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable13((cablesWithTags[13]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable14((cablesWithTags[14]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable15((cablesWithTags[15]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable16((cablesWithTags[16]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable17((cablesWithTags[17]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable18((cablesWithTags[18]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable19((cablesWithTags[19]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable20((cablesWithTags[20]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable21((cablesWithTags[21]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable22((cablesWithTags[22]->getHistory()).tensionHistory);
  std::deque<double> tensionHistoryCable23((cablesWithTags[23]->getHistory()).tensionHistory);
  
  FILE *pFile;
  std::string fileNameStr = "/home/tensegribuntu/projects/tg_shared/";
  fileNameStr += timebuffer;
  fileNameStr += "_histLogTension.csv";
  const char* fileName = fileNameStr.c_str();
  pFile = fopen(fileName,"a");
  if(pFile!=NULL){
    for(int i=0;i<tensionHistoryCable0.size();i++)
    {
      double t = (double)i/1000;
      fprintf(pFile, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",t,
                      tensionHistoryCable0[i],
                      tensionHistoryCable1[i],
                      tensionHistoryCable2[i],
                      tensionHistoryCable3[i],
                      tensionHistoryCable4[i],
                      tensionHistoryCable5[i],
                      tensionHistoryCable6[i],
                      tensionHistoryCable7[i],
                      tensionHistoryCable8[i],
                      tensionHistoryCable9[i],
                      tensionHistoryCable10[i],
                      tensionHistoryCable11[i],
                      tensionHistoryCable12[i],
                      tensionHistoryCable13[i],
                      tensionHistoryCable14[i],
                      tensionHistoryCable15[i],
                      tensionHistoryCable16[i],
                      tensionHistoryCable17[i],
                      tensionHistoryCable18[i],
                      tensionHistoryCable19[i],
                      tensionHistoryCable20[i],
                      tensionHistoryCable21[i],
                      tensionHistoryCable22[i],
                      tensionHistoryCable23[i]);                      
    }
    fclose(pFile);
  }
}