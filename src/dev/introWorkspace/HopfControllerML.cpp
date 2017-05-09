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

/*#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
using namespace boost::numeric::odeint;
typedef boost::array<double,3> state_type;*/
//const double sigma = 10;
//const double R = 28;
//const double b = 8/3;

#include <math.h>
//#include <boost/array.hpp>
//#include <boost/numeric/odeint.hpp>
#include <fstream>
#include <cstdio>
#include <time.h>
#include <string.h>
#include <stdlib.h>

//using namespace std;
//using namespace boost::numeric::odeint;
//typedef boost::array<double,2> state_type2;
//namespace pl = std::placeholders;

const double mu=0.7;
const double omega=5.0;
//const char *csvpath="/home/tensegribuntu/projects/tg_shared/hopf_results.csv";
//const int i=0;
char filenamehopf[70];
char filenamecable[80];
char filenamespeed[80];
double rand_perturbation0 = 0.0;
double rand_perturbation1 = 0.0;

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
HopfControllerML::HopfControllerML(double startTime,
					   double minLength,
					   double rate,
					   std::vector<std::string> tagsToControl) :
  m_startTime(startTime),
  m_minLength(minLength),
  m_rate(rate),
  m_tagsToControl(tagsToControl),
  m_timePassed(0.0),
  hopfArray({0.1,0.1}),
  dxdt({0.0,0.0}),
  ddxddt({0.0,0.0}),
  /*hopf_x(0.11),
  hopf_y(0.34),*/
  hopf_omega(2.1),//0.56),
  hopf_mu(0.78),//0.78),
  ctr(0),
  initRestLengths(0) //x({10,1,1})
{
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
 * The initializeActuators method is call in onSetup to put pointers to 
 * specific actuators in the cablesWithTags array, as well as store the initial
 * rest lengths in the initialRL map.
 */
void HopfControllerML::initializeActuators(TensegrityModel& subject,
					       std::string tag) 
{
  //DEBUGGING
  std::cout << "Finding cables with the tag: " << tag << std::endl;
  // Pick out the actuators with the specified tag
  //std::vector<tgBasicActuator*> foundActuators = subject.find<tgBasicActuator>(tag);
  std::vector<tgKinematicActuator*> foundActuators = subject.find<tgKinematicActuator>(tag);
  std::cout << "The following cables were found and will be controlled: "
	    << std::endl;
  //Iterate through array and output strings to command line
  for (std::size_t i = 0; i < foundActuators.size(); i ++) {	
    std::cout << foundActuators[i]->getTags() << std::endl;
    // Also, add the rest length of the actuator at this time
    // to the list of all initial rest lengths.
    initialRL[foundActuators[i]->getTags()] = foundActuators[i]->getRestLength();
    //DEBUGGING:
    std::cout << "Cable rest length at t=0 is "
	      << initialRL[foundActuators[i]->getTags()] << std::endl;
  }
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  cablesWithTags.insert( cablesWithTags.end(), foundActuators.begin(),
			 foundActuators.end() );
}

/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void HopfControllerML::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up the HopfControllerML controller." << std::endl;
  //	    << "Finding cables with tags: " << m_tagsToControl
  //	    << std::endl;
  cablesWithTags = {};
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for( it = m_tagsToControl.begin(); it < m_tagsToControl.end(); it++ ) {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }
  std::cout << "Finished setting up the controller." << std::endl;    
}

void HopfControllerML::onStep(TensegrityModel& subject, double dt)
{
  m_timePassed += dt;

  if(m_timePassed > 3000*dt&&m_timePassed<30000*dt){
    if(ctr==0){
      initRestLengths=cablesWithTags[0]->getRestLength();

      srand(time(NULL));

      time_t rawtime;
      struct tm *timeinfo;
      char timebuffer[12];
      time(&rawtime);
      timeinfo = localtime(&rawtime);
      strftime(timebuffer,12,"%m_%d_%H_%M",timeinfo);
      strcpy(filenamehopf,"/home/tensegribuntu/projects/tg_shared/hopf_results_");
      strcat(filenamehopf,timebuffer);
      strcat(filenamehopf,".csv");

      time_t rawtime2;
      struct tm *timeinfo2;
      char timebuffer2[12];
      time(&rawtime2);
      timeinfo2 = localtime(&rawtime2);
      strftime(timebuffer2,12,"%m_%d_%H_%M",timeinfo);
      strcpy(filenamecable,"/home/tensegribuntu/projects/tg_shared/cable_hopf_results_");
      strcat(filenamecable,timebuffer2);
      strcat(filenamecable,".csv");

      time_t rawtime3;
      struct tm *timeinfo3;
      char timebuffer3[12];
      time(&rawtime3);
      timeinfo3 = localtime(&rawtime3);
      strftime(timebuffer3,12,"%m_%d_%H_%M",timeinfo);
      strcpy(filenamespeed,"/home/tensegribuntu/projects/tg_shared/speed_hopf_results_");
      strcat(filenamespeed,timebuffer3);
      strcat(filenamespeed,".csv");
    }
    ctr++;
    //cablesWithTags[1]->setControlInput(-50000);
    //cablesWithTags[1]->setControlInput(-50000);
    //std::cout<<"initRestLengths: "<<initRestLengths<<std::endl; 
    //sineTest(subject,m_timePassed,0,1,initRestLengths,0,-0.05);
    //sineTest(subject,m_timePassed,0,6,initRestLengths,-0.333*3.14,0.15);
    /*sineTest(subject,m_timePassed, 0, 4, initRestLengths, 0, 0.3);
    sineTest(subject,m_timePassed, 4, 8, initRestLengths, 2*0.333*3.14, 0.3);
    sineTest(subject,m_timePassed, 8, 12, initRestLengths, 4*0.333*3.14, 0.3);*/
    hopfController(subject, hopfArray, dt, m_timePassed, dxdt, ddxddt, 0, 12, initRestLengths,0);
    hopfController(subject, hopfArray, dt, m_timePassed, dxdt, ddxddt, 12, 24, initRestLengths,1);
    /*if(m_timePassed>=7000*dt && m_timePassed<10000*dt){
      //std::cout << "PERTURBATION" << std::endl;
      cablesWithTags[0]->setControlInput(25,dt); 
    }*/
    //std::cout << "hopfArray: " << hopfArray[0] << " " << hopfArray[1] << std::endl;
  }
  if(m_timePassed <= 30001*dt){
    //write_hopf(hopfArray, m_timePassed, filenamehopf, filenamecable, filenamespeed);
    if(m_timePassed>30000*dt){
      std::cout << "DONE" << std::endl;
    }
  }
  if(m_timePassed>30000*dt){
    for(int i=0; i<24; i++){
      cablesWithTags[i]->setControlInput(0);
    }
  }
}

void HopfControllerML::sineTest(TensegrityModel& subject, double dt, int firstCable, int lastCable, double initRestLengths, double offset, double amp)
{
  double sin_res = 0;
  //sin_res=(sin(0.5*dt)+1)/2;
  sin_res = amp*(sin(2*3.14*0.2*dt+offset)-1);
  //std::cout<<"sin_res: "<<sin_res<<std::endl;
 
  for (std::size_t i = firstCable; i < lastCable; i ++) {  
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

void HopfControllerML::checkLengths(TensegrityModel& subject, double dt, int firstCable, int lastCable)
{
  // For each cable, check if its rest length is past the minimum,
  // otherwise adjust its length according to m_rate and dt.
  //for (std::size_t i = 0; i < cablesWithTags.size()/3; i ++) {  
  for (std::size_t i = firstCable; i < lastCable; i ++) {  
    double currRestLength = cablesWithTags[i]->getRestLength();
    // Calculate the minimum rest length for this cable.
    // Remember that m_minLength is a percent.
    double minRestLength = initialRL[cablesWithTags[i]->getTags()] * m_minLength;
    // If the current rest length is still greater than the minimum,qqq
    if( currRestLength > minRestLength ) {
      // output a progress bar for the controller, to track when control occurs.
      //std::cout << "." << i;

      //std::cout << "Rest Length = " << currRestLength << std::endl;  

      // Then, adjust the rest length of the actuator itself, according to
      // m_rate and dt.
      double nextRestLength = currRestLength - m_rate * dt;
      //DEBUGGING
      //std::cout << "Next Rest Length: " << nextRestLength << std::endl;
      //cablesWithTags[i]->setControlInput(nextRestLength,dt);
      cablesWithTags[i]->setControlInput(nextRestLength);
    }
  }
}

void HopfControllerML::hopfController(TensegrityModel& subject, double *hopfArray, double dt, double m_timePassed, double *dxdt, double *ddxddt, int firstCable, int lastCable, double initRestLengths, int hopfSelector)//, double *hopf_x, double hopf_y, double hopf_omega, double hopf_mu)
{
  double r = sqrt(hopfArray[0]*hopfArray[0] + hopfArray[1]*hopfArray[1]);

  dxdt[0] = (hopf_mu-r*r)*hopfArray[0] - hopf_omega*hopfArray[1];
  dxdt[1] = (hopf_mu-r*r)*hopfArray[1] + hopf_omega*hopfArray[0];

  double drdt = (hopfArray[0]*dxdt[0] + hopfArray[1]*dxdt[1])/r;

  ddxddt[0] = -2*r*drdt*hopfArray[0] + (hopf_mu-r*r)*dxdt[0] - hopf_omega*dxdt[1];
  ddxddt[1] = -2*r*drdt*hopfArray[1] + (hopf_mu-r*r)*dxdt[1] + hopf_omega*dxdt[0];

  hopfArray[0] = hopfArray[0] + dxdt[0]*dt + 0.5*ddxddt[0]*dt*dt;
  hopfArray[1] = hopfArray[1] + dxdt[1]*dt + 0.5*ddxddt[1]*dt*dt;

  for (std::size_t i = firstCable; i < lastCable; i ++) {  
    /*//std::cout<<*cablesWithTags[i]<<std::endl;
    //double currRestLength=cablesWithTags[i]->getRestLength();
    //std::cout<<"currRestLength: "<<currRestLength<<std::endl;
    double buffer_var = initRestLengths;
    if(hopfSelector==0){
      buffer_var = initRestLengths*(1+hopfArray[0]);
    }
    else{
      buffer_var = initRestLengths*(1+hopfArray[1]);
    }
    
    double minRestLength = initialRL[cablesWithTags[i]->getTags()] * m_minLength;
    //std::cout << "minRestLength: " << minRestLength << std::endl;

    double nextRestLength = initRestLengths;
    if(buffer_var < minRestLength){
      nextRestLength = minRestLength;//(1+sqrt(hopfArray[0]*hopfArray[0]+hopfArray[1]*hopfArray[1]));
      //std::cout << "Ctl input: " << nextRestLength << ", velocity: " << cablesWithTags[i]->getVelocity() << std::endl;
    }
    else if(buffer_var > 1.8*initRestLengths){
      nextRestLength = 1.8*initRestLengths;
      //std::cout << "Ctl input: " << nextRestLength << ", velocity: " << cablesWithTags[i]->getVelocity() << std::endl;
    }
    else{
      nextRestLength = buffer_var;
      //std::cout << "Ctl input: " << nextRestLength << ", velocity: " << cablesWithTags[i]->getVelocity() << std::endl;
    }
 */
    double buffer_var = 0;
    if(hopfSelector==0){
      buffer_var = hopfArray[0];
    }
    else{
      buffer_var = hopfArray[1];
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
    if(m_timePassed>2000*dt){
      //std::cout << "Factor: " << 30-m_timePassed << std::endl;
      cablesWithTags[i]->setControlInput((30-m_timePassed)/30*5000*buffer_var);
    }
    else
      cablesWithTags[i]->setControlInput(5000*buffer_var);

  }
}

/*void HopfControllerML::hopf(const state_type2 &x, state_type2 &dxdt, double t)
{
    double r = 0;

    r = sqrt(x[0]*x[0] + x[1]*x[1]);
    dxdt[0] = (mu-r*r)*x[0] - omega*x[1];
    dxdt[1] = (mu-r*r)*x[1] + omega*x[0];
}*/

void HopfControllerML::write_hopf(double *hopfArray, double t, char *filenamehopf, char *filenamecable, char *filenamespeed)
{
  FILE *pFile;
  pFile = fopen(filenamehopf,"a");
  if(pFile!=NULL){
    fprintf(pFile, "%f,%f,%f\n",t,hopfArray[0],hopfArray[1]);
    fclose(pFile);
  }

  double buffer_var = initRestLengths*(1+hopfArray[0]);
  FILE *pFile2;
  pFile2 = fopen(filenamecable,"a");
  if(pFile2!=NULL){
    fprintf(pFile2, "%f,%f,%f\n",t, buffer_var, cablesWithTags[0]->getRestLength());
    fclose(pFile2);
  }

  FILE *pFile3;
  pFile3 = fopen(filenamespeed,"a");
  if(pFile3!=NULL){
    fprintf(pFile3, "%f,%f\n",t, cablesWithTags[0]->getVelocity());
    fclose(pFile3);
  }
}