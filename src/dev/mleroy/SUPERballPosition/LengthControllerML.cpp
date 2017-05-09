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
 * @file LengthControllerML.cpp
 * @brief Implementation of LengthControllerML.
 * @author Marc Leroy
 * $Id$
 */

// This module
#include "LengthControllerML.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
// This library
#include "core/tgBasicActuator.h"
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
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
//#include <fstream>
//using namespace std;
//using namespace boost::numeric::odeint;
typedef boost::array<double,2> state_type2;
//namespace pl = std::placeholders;

const double mu=0.7;
const double omega=5.0;
//const char *csvpath="/home/tensegribuntu/projects/tg_shared/hopf_results.csv";
//const int i=0;


// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
LengthControllerML::LengthControllerML(double startTime,
					   double minLength,
					   double rate,
					   std::vector<std::string> tagsToControl) :
  m_startTime(startTime),
  m_minLength(minLength),
  m_rate(rate),
  m_tagsToControl(tagsToControl),
  m_timePassed(0.0),
  hopf_x(0.11),
  hopf_y(0.34),
  hopf_omega(0.56),
  hopf_mu(0.78),
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
void LengthControllerML::initializeActuators(TensegrityModel& subject,
					       std::string tag) {
  //DEBUGGING
  std::cout << "Finding cables with the tag: " << tag << std::endl;
  // Pick out the actuators with the specified tag
  std::vector<tgBasicActuator*> foundActuators = subject.find<tgBasicActuator>(tag);
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
void LengthControllerML::onSetup(TensegrityModel& subject)
{
  std::cout << "Setting up the LengthControllerML controller." << std::endl;
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

void LengthControllerML::onStep(TensegrityModel& subject, double dt)
{
  // First, increment the accumulator variable.
  m_timePassed += dt;
  //std::cout << "Current time: " << m_timePassed << std::endl; 
  // Then, if it's passed the time to start the controller,
  /*if( m_timePassed > m_startTime ) {
    checkLengths(subject, dt, 0, 2); 
  }

  if( m_timePassed > 6000*dt ) {
    checkLengths(subject, dt, 2, 4);
  }

  if( m_timePassed > 10000*dt ) {
    checkLengths(subject, dt, 4, 6);
  }

  if(m_timePassed > 1400*dt && m_timePassed < 1455*dt){
    //hopf_x = 2*hopf_x;
    //hopfController(subject,&hopf_x,hopf_y,hopf_omega,hopf_mu);
  }
  */
  if(m_timePassed > 3000*dt){//&&m_timePassed<3010*dt){
    if(ctr==0){
      std::cout<<"?!?!?"<<std::endl;
      initRestLengths=cablesWithTags[0]->getRestLength();
    }
    ctr++;
    //std::cout<<"initRestLengths: "<<initRestLengths<<std::endl; 
    //sineTest(subject,m_timePassed,0,1,initRestLengths,0,-0.05);
    //sineTest(subject,m_timePassed,0,6,initRestLengths,-0.333*3.14,0.15);
    sineTest(subject,m_timePassed, 0, 4, initRestLengths, 0, 0.3);
    sineTest(subject,m_timePassed, 4, 8, initRestLengths, 2*0.333*3.14, 0.3);
    sineTest(subject,m_timePassed, 8, 12, initRestLengths, 4*0.333*3.14, 0.3);
  }
}

void LengthControllerML::sineTest(TensegrityModel& subject, double dt, int firstCable, int lastCable, double initRestLengths, double offset, double amp)
{
  double sin_res = 0;
  //sin_res=(sin(0.5*dt)+1)/2;
  sin_res = amp*sin(2*3.14*0.2*dt+offset);
  //std::cout<<"sin_res: "<<sin_res<<std::endl;
 
  for (std::size_t i = firstCable; i < lastCable; i ++) {  
    //std::cout<<*cablesWithTags[i]<<std::endl;
    //double currRestLength=cablesWithTags[i]->getRestLength();
    //std::cout<<"currRestLength: "<<currRestLength<<std::endl;
    double nextRestLength = initRestLengths*(1+sin_res);
    //std::cout<<"nextRestLength: "<<nextRestLength<<std::endl;
    //std::cout<<std::endl;
    cablesWithTags[i]->setControlInput(nextRestLength,dt);
  }
}

void LengthControllerML::checkLengths(TensegrityModel& subject, double dt, int firstCable, int lastCable)
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
      cablesWithTags[i]->setControlInput(nextRestLength,dt);
    }
  }
}

void LengthControllerML::hopfController(TensegrityModel& subject, double *hopf_x, double hopf_y, double hopf_omega, double hopf_mu)
{
  std::cout << "In Hopf Controller function" << std::endl;
  std::cout << "x = " << *hopf_x << ", y = " << hopf_y << ", omega = " << hopf_omega << ", mu = " << hopf_mu << std::endl;
  *hopf_x *= 2;
  if(*hopf_x==0.44)
  {
    test_fct();
    //write_lorenz(x, 0);
    //state_type x = {10.0,1.0,1.0};
    //integrate(lorenz, x, 0.0, 25.0, 0.1, write_lorenz);
    //integrate({10*(x[1]-x[0]),28*x[0] - x[1] - x[2]*x[0], -8.0/3.0*x[2] + x[0]*x[1]},0.0,5.0,0.1,std::cout << t << "\t" << x[0] << "\t" << x[1] << "\t" << x[2] << std::endl);
  }


}

void LengthControllerML::hopf(const state_type2 &x, state_type2 &dxdt, double t)
{
    double r = 0;

    r = sqrt(x[0]*x[0] + x[1]*x[1]);
    dxdt[0] = (mu-r*r)*x[0] - omega*x[1];
    dxdt[1] = (mu-r*r)*x[1] + omega*x[0];
}

void LengthControllerML::write_hopf(const state_type2 &x, const double t)
{
    std::cout << t << '\t' << x[0] << '\t' << x[1] << std::endl;
    /*ofstream myfile(csvpath);
    myfile.open(csvpath,std::ios_base::app);
    myfile << t << "," << x[0] << "," << x[1] << endl;
    myfile.close();*/
}

void LengthControllerML::test_fct()
{
    std::cout << "Mine turtle TEST" << std::endl;
    /*state_type2 x_h2d = {0.25,0.25};
    //boost::numeric::odeint::integrate(&LengthControllerML::hopf->this,x_h2d,0.0,10.0,0.1,write_hopf);
    hopf=[this](const state_type2 &x, state_type2 &dxdt, double t){
      this->hopf(x,dxdt,t);
    };
    write_hopf=[this](const state_type2 &x, double t){
      this->write_hopf(x,t);
    };
    boost::numeric::odeint::integrate(hopf,x_h2d,0.0,10.0,0.1,write_hopf);*/
}

/*void LengthControllerML::lorenz(const state_type &x, state_type &dxdt, double t)
{
  double sigma = 10;
  double R = 28;
  double b = 8/3;
  std::cout << "Hello TEST" << std::endl;  
    dxdt[0] = sigma*(x[1]-x[0]);
    dxdt[1] = R*x[0] - x[1] - x[2]*x[0];
    dxdt[2] = -b*x[2] + x[0]*x[1];
}

void LengthControllerML::write_lorenz(const state_type &x, const double t)
{
    std::cout << t << "\t" << x[0] << "\t" << x[1] << "\t" << x[2] << std::endl;
}*/