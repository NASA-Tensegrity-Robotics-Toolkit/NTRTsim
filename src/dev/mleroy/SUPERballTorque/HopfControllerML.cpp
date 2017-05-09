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
//#include "core/tgBasicActuator.h"
#include "core/tgKinematicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"

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
static std::string hopfFileName;
static std::string speedFileName;
static std::string cableFileName;
static std::vector<std::string> fileNames;

const double CONTROLLER_STOP_TIME = 30000.0;
const double HOPF_AMPLIFIER = 500.0;

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
HopfControllerML::HopfControllerML(double startTime, double minLength, double rate, std::vector<std::string> tagsToControl, double timePassed, 
                                   int ctr, double initRestLengths, double hopfOmega, double hopfMu, double hopfStateInit[2], double hopfVelInit[2], double hopfAccInit[2]) :
  m_startTime(startTime),
  m_minLength(minLength),
  m_rate(rate),
  m_tagsToControl(tagsToControl),
  m_timePassed(timePassed),
  ctr(ctr),
  initRestLengths(initRestLengths),
  hopfOmega(hopfOmega),//0.56),
  hopfMu(hopfMu)//0.78)
  //hopfState(hopfState),//{0.1,0.1}),
  //hopfVel(hopfVel),//{0.0,0.0}),
  //hopfAcc(hopfAcc)//{0.0,0.0}),
{
  hopfState[0] = hopfStateInit[0];
  hopfState[1] = hopfStateInit[1];
  hopfVel[0] = hopfVelInit[0];
  hopfVel[1] = hopfVelInit[1];
  hopfAcc[0] = hopfAccInit[0];
  hopfAcc[1] = hopfAccInit[1];

  hopfFileName = setupCSVFiles(primerHopfFileName);
  speedFileName = setupCSVFiles(primerSpeedFileName);
  cableFileName = setupCSVFiles(primerCableFileName);

  fileNames.push_back(hopfFileName);
  fileNames.push_back(speedFileName);
  fileNames.push_back(cableFileName);
  //std::cout << fileNames[0] << " " << fileNames[1] << " " << fileNames[2] << std::endl;
  /*
  std::cout << hopfState[0] << ", " << hopfState[1] << std::endl;
  std::cout << hopfVel[0] << ", " << hopfVel[1] << std::endl;
  std::cout << hopfAcc[0] << ", " << hopfAcc[1] << std::endl;
  */

  // start time must be greater than or equal to zero
  if( m_startTime < 0.0 ) {
    throw std::invalid_argument("Start time must be greater than or equal to zero.");
  }
  // min length must be between 1 and 0
  else if( m_minLength > 1 ) {
    throw std::invalid_argument("minLength is a percent, must be less than 1. (100%)");
  }
  else if( m_minLength < 0.0) {
    throw std::invalid_argument("minLength is a percent, must be greater than 0.");
  }
  // rate must be greater than zero
  else if( rate < 0.0 ) {
    throw std::invalid_argument("Rate cannot be negative.");
  }
  // @TODO: what checks to make on tags?
}


/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void HopfControllerML::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up the HopfControllerML controller." << std::endl;
  //      << "Finding cables with tags: " << m_tagsToControl
  //      << std::endl;
  cablesWithTags = {};
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for( it = m_tagsToControl.begin(); it < m_tagsToControl.end(); it++ ) {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }
  std::cout << "Finished setting up the controller." << std::endl;
}


/**
 * The initializeActuators method is call in onSetup to put pointers to 
 * specific actuators in the cablesWithTags array, as well as store the initial
 * rest lengths in the initialRL map.
 */
void HopfControllerML::initializeActuators(TensegrityModel& subject, std::string tag) 
{
  //DEBUGGING
  std::cout << "Finding cables with the tag: " << tag << std::endl;
  // Pick out the actuators with the specified tag
  //std::vector<tgBasicActuator*> foundActuators = subject.find<tgBasicActuator>(tag);
  std::vector<tgKinematicActuator*> foundActuators = subject.find<tgKinematicActuator>(tag);
  
  std::cout << "The following cables were found and will be controlled: " << std::endl;
  //Iterate through array and output strings to command line
  for (std::size_t i = 0; i < foundActuators.size(); i ++) 
  {	
    std::cout << foundActuators[i]->getTags() << std::endl;
    // Also, add the rest length of the actuator at this time
    // to the list of all initial rest lengths.
    initialRL[foundActuators[i]->getTags()] = foundActuators[i]->getRestLength();
    //initialRLArray.push_back(foundActuators[i]->getRestLength()); //ML
    //DEBUGGING:
    std::cout << "Cable rest length at t=0 is " << initialRL[foundActuators[i]->getTags()] << std::endl;
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
    
  if(m_timePassed > 3000*dt && m_timePassed < CONTROLLER_STOP_TIME*dt)
  {
    if(ctr==0)
    {
      initRestLengths=cablesWithTags[0]->getRestLength();
      for(InitialRestLengths::const_iterator it = initialRL.begin(); it != initialRL.end(); ++it)
      {
        std::cout << it->first << " " << it->second << std::endl;
      }
      //std::cout << initRestLengths << std::endl;
      //std::cout << "Test length: " << initialRL.at((tgTags)"SUPERball_string21") << std::endl;
    }
    ctr++;
      
    /*for(InitialRestLengths::const_iterator it = initialRL.begin(); it != initialRL.end(); ++it)
    {
      std::cout << it->first << " " << it->second << std::endl;
    }
    std::cout << std::endl;*/

    hopfOscillator(subject, dt, m_timePassed, hopfState, hopfVel, hopfAcc, 0, 11, initRestLengths,0);
    hopfOscillator(subject, dt, m_timePassed, hopfState, hopfVel, hopfAcc, 12, 23, initRestLengths,1);
   
    /*
    std::deque<double> testDeque((cablesWithTags[0]->getHistory()).lastLengths);
    //std::cout << "The history has after " << 1000*m_timePassed << ":" << std::endl;
    //for(std::deque<double>::iterator it = testDeque.begin(); it!=testDeque.end(); ++it)
      //std::cout << " " << *it;
    //std::cout << "AND " << std::endl;
    std::cout << testDeque[1000*m_timePassed] << std::endl;
    //std::cout << "NEXT" << std::endl;
    */
  }

  if(m_timePassed <= (1+CONTROLLER_STOP_TIME)*dt)
  {
    //exportHopfCSV(m_timePassed, hopfState, fileNames);
    if(m_timePassed>CONTROLLER_STOP_TIME*dt)
    {
      std::cout << "DONE" << std::endl;      
      if(0)
      {
        saveHistLastLengths();
        saveHistRestLengths();
        saveHistDamping();
        saveHistLastVelocities();
        saveHistTension();
      }
    }
  }

  /*if(m_timePassed>CONTROLLER_STOP_TIME*dt)
  {
    for(int i=0; i<=23; i++)
    {
      cablesWithTags[i]->setControlInput(0);
    }
  }*/
}


/**
 * Test method for position control (/!\ requires tgBasicActuator class, not tgKinematicActuator)
 */
void HopfControllerML::sineTest(TensegrityModel& subject, double dt, int firstCable, int lastCable, double initRestLengths, double offset, double amp)
{
  double sin_res = 0;
  //sin_res=(sin(0.5*dt)+1)/2;
  sin_res = amp*(sin(2*3.14*0.2*dt+offset)-1);
  //std::cout<<"sin_res: "<<sin_res<<std::endl;
 
  for (std::size_t i = firstCable; i < lastCable; i ++) 
  {  
    //std::cout<<*cablesWithTags[i]<<std::endl;
    //double currRestLength=cablesWithTags[i]->getRestLength();
    //std::cout<<"currRestLength: "<<currRestLength<<std::endl;
    double nextRestLength = initRestLengths*(1+sin_res);
    //std::cout<<"nextRestLength: "<<nextRestLength<<std::endl;
    //std::cout<<std::endl;
    //cablesWithTags[i]->setControlInput(nextRestLength,dt);
    cablesWithTags[i]->setControlInput(nextRestLength);
  }
}


/**
 * Test method for position control (/!\ requires tgBasicActuator class, not tgKinematicActuator)
 */
void HopfControllerML::checkLengths(TensegrityModel& subject, double dt, int firstCable, int lastCable)
{
  // For each cable, check if its rest length is past the minimum,
  // otherwise adjust its length according to m_rate and dt.
  for (std::size_t i = firstCable; i < lastCable; i ++)
  {  
    double currRestLength = cablesWithTags[i]->getRestLength();
    
    // Calculate the minimum rest length for this cable.
    // Remember that m_minLength is a percent.
    double minRestLength = initialRL[cablesWithTags[i]->getTags()] * m_minLength;
    
    // If the current rest length is still greater than the minimum,
    if( currRestLength > minRestLength ) 
    {
      // Then, adjust the rest length of the actuator itself, according to
      // m_rate and dt.
      double nextRestLength = currRestLength - m_rate * dt;
    
      cablesWithTags[i]->setControlInput(nextRestLength);
    }
  }
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
void HopfControllerML::hopfOscillator(TensegrityModel& subject, double dt, double m_timePassed, double *hopfState, double *hopfVel, double *hopfAcc,
                                      int firstCable, int lastCable, double initRestLengths, int hopfSelector)//, double hopfOmega, double hopfMu)
{
  //std::cout << "In function hopfOscillator" << std::endl;
  //std::cout << hopfState[0] << " " << hopfState[1] << " " << hopfVel[0] << " " << hopfVel[1] << " " << hopfAcc[0] << " " << hopfAcc[1] << " " << m_timePassed << std::endl;
  compNextHopfState(dt);
  
  for (std::size_t i = firstCable; i <= lastCable; i ++) 
  {  
    double buffer_var = 0;
    
    if(hopfSelector==0)
    {
      buffer_var = hopfState[0];
    }
    else
    {
      buffer_var = hopfState[1];
    }
    /*if(m_timePassed>=8000*dt && m_timePassed<25000*dt){
      //std::cout << "PERTURBATION" << std::endl;
      if((int)(1000*m_timePassed)%250==0){
        if(hopfSelector==0){
          rand_perturbation0 = (double)rand()/RAND_MAX-0.5;
        }
        else{
          rand_perturbation1 = (double)rand()/RAND_MAX-0.5;
        }
      }
      if(hopfSelector==0){    
        cablesWithTags[i]->setControlInput(nextRestLength+rand_perturbation0,dt);
      }
      else{
        cablesWithTags[i]->setControlInput(nextRestLength+rand_perturbation1,dt); 
      }
      //std::cout << "0: " << rand_perturbation0 << ", 1: " << rand_perturbation1 << std::endl;
    }
    else{*/
      //std::cout<<"nextRestLength: "<<nextRestLength<<std::endl;
      //std::cout<<std::endl;
//      cablesWithTags[i]->setControlInput(nextRestLength,dt);
      /*if(m_timePassed>=26000*dt && m_timePassed<=26100*dt){
        //std::cout << "Applying negative torque" << std::endl;
        cablesWithTags[1]->setControlInput(-100000.0);
      }*/
      //if(m_timePassed>=5000*dt){// && m_timePassed<=14500*dt){
        //std::cout << "Applying positive torque" << std::endl;
        //std::cout << 100*buffer_var << std::endl;
        //cablesWithTags[i]->setControlInput(5000*buffer_var);
      //}
      //std::cout << "Ctl input: " << nextRestLength << ", velocity: " << cablesWithTags[i]->getVelocity() << std::endl;
    //}
    
    /*if(m_timePassed>10000*dt)
    {
      //std::cout << "Factor: " << 30-m_timePassed << std::endl;
      cablesWithTags[i]->setControlInput((CONTROLLER_STOP_TIME-1000*m_timePassed)/CONTROLLER_STOP_TIME*HOPF_AMPLIFIER*buffer_var);
    }
    else
    {*/
      cablesWithTags[i]->setControlInput(HOPF_AMPLIFIER*buffer_var);
    //}

  }
}


/**
 * Computes the next iteration of the Hopf oscillator state thanks to the Euler method
 * @param[in] dt the time step used for the numerical derivation
 * @return void
 */
void HopfControllerML::compNextHopfState(double dt)
{
  //std::cout << "In function compNextHopfState" << std::endl;
  double r = sqrt(hopfState[0]*hopfState[0] + hopfState[1]*hopfState[1]);

  hopfVel[0] = (hopfMu-r*r)*hopfState[0] - hopfOmega*hopfState[1];
  hopfVel[1] = (hopfMu-r*r)*hopfState[1] + hopfOmega*hopfState[0];

  double drdt = (hopfState[0]*hopfVel[0] + hopfState[1]*hopfVel[1])/r;

  hopfAcc[0] = -2*r*drdt*hopfState[0] + (hopfMu-r*r)*hopfVel[0] - hopfOmega*hopfVel[1];
  hopfAcc[1] = -2*r*drdt*hopfState[1] + (hopfMu-r*r)*hopfVel[1] + hopfOmega*hopfVel[0];

  hopfState[0] = hopfState[0] + hopfVel[0]*dt + 0.5*hopfAcc[0]*dt*dt;
  hopfState[1] = hopfState[1] + hopfVel[1]*dt + 0.5*hopfAcc[1]*dt*dt;

  //std::cout << hopfState[0] << " " << hopfState[1] << " " << hopfVel[0] << " " << hopfVel[1] << " " << hopfAcc[0] << " " << hopfAcc[1] << " " << m_timePassed << std::endl;
}


/**
 * Setup of the timestamped filenames that will be used to export data in .csv format 
 * @param[in] fileDataType string describing start of file's name, e.g. speed or tension
 * @return outputFileName string of full path towards timestamped file
 */
std::string HopfControllerML::setupCSVFiles(std::string fileDataType)
{  
  srand(time(NULL));
  time_t rawtime;
  struct tm *timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(timebuffer,12,"%m_%d_%H_%M",timeinfo);
  std::string outputFileName = "/home/tensegribuntu/projects/tg_shared/" + (std::string)timebuffer + fileDataType + ".csv";
 
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
void HopfControllerML::exportHopfCSV(double t, double *hopfState, std::vector<std::string> fileNames)
{
  FILE *pFile;
  const char* file1Name = fileNames[0].c_str();
  pFile = fopen(file1Name,"a");
  if(pFile!=NULL){
    fprintf(pFile, "%f,%f,%f\n",t,hopfState[0],hopfState[1]);
    fclose(pFile);
  }

  FILE *pFile2;
  const char* file2Name = fileNames[1].c_str();
  pFile2 = fopen(file2Name,"a");
  if(pFile2!=NULL){
    fprintf(pFile2, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",t, 
                      cablesWithTags[0]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[1]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[2]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[3]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[4]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[5]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[6]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[7]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[8]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[9]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[10]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[11]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[12]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[13]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[14]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[15]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[16]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[17]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[18]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[19]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[20]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[21]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[22]->tgKinematicActuator::getVelocity(),
                      cablesWithTags[23]->tgKinematicActuator::getVelocity());
    fclose(pFile2);
  }

  double buffer_var = initRestLengths*(1+hopfState[0]);
  FILE *pFile3;
  const char* file3Name = fileNames[2].c_str();
  pFile3 = fopen(file3Name,"a");
  if(pFile3!=NULL){
    fprintf(pFile3, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",t, buffer_var,
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