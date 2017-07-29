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
 * @file LengthController.cpp
 * @brief Implementation of class LengthController
 * @author Brian Cera
 * $Id$
 */

// This module
#include "LengthController.h"
// This application
#include "T6Model.h"
#include "core/tgBasicActuator.h"
#include "core/tgSpringCable.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

using namespace std;

LengthController::LengthController(const double length) :
  m_length(length)
{
  if (length < 0.0)
    {
      throw std::invalid_argument("Negative length");
    }
}

LengthController::~LengthController()
{
}	

void LengthController::onSetup(T6Model& subject)
{

  m_controllers.clear(); //clear vector of controllers
  
  //get all of the tensegrity structure's cables
  actuators = subject.getAllActuators();

  //Attach a tgBasicController to each actuator
  for (size_t i = 0; i < actuators.size(); ++i)
    {
      tgBasicActuator * const pActuator = actuators[i];
      assert(pActuator != NULL);
      //instantiate controllers for each cable
      tgBasicController* m_lenController = new tgBasicController(pActuator, m_length);
      //add controller to vector
      m_controllers.push_back(m_lenController);
    }
    
    globalTime = 0;
}

void LengthController::onStep(T6Model& subject, double dt)
{
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
        return;
    }
    globalTime += dt;
    
    // Manifold control variables:
    double l = 16.84;
    double b = l*sqrt(3.0/8.0);
    //b = 0.2*l;
    //double delta = acos(1.0/sqrt(3.0));
    double alpha = M_PI/3.0;
    //alpha = 58.0/180.0*M_PI;
    
    // delta as a function of time:
    double delta = (55.0-88.0)*(globalTime-2.0)/5.0 + 88.0;
    //alpha = (50.0-58.0)*(globalTime-2.0)/5.0 + 58.0;

    //if (alpha < 50.0) alpha = 50.0;
    //if (alpha > 58.0) alpha = 58.0;
    if (delta < 55.0) delta = 55.0;
    if (delta > 88.0) delta = 88.0;
    //alpha = 60; delta=55;
    
    alpha = 62.0/180.0 * M_PI;
    delta = delta/180.0 * M_PI; // rad
    
    // Compute the control SVDB tensions from Skelton, 2001:
    double u = sin(delta) * cos(alpha + M_PI/6.0);
    double h = cos(delta)/(2.0*u) * (l*u + sqrt(b*b/3.0 - 3.0*l*l*u*u) - b/sqrt(3.0));
    
    double S = sqrt(h*h + b*b/3.0 + l*l*sin(delta)*sin(delta)
               - 2.0/sqrt(3.0)*l*b*sin(delta)*cos(alpha - M_PI/6.0) );
    double V = sqrt(b*b + l*l - 2.0*l*b*sin(delta)*sin(alpha + M_PI/6.0));
    double D = sqrt(h*h + b*b/3.0 + l*l - 2.0/sqrt(3.0)*l*b*sin(delta)*sin(alpha) - 2.0*l*h*cos(delta));
    
    double T_V = V/D * 1.0 / (sqrt(3.0)*cos(alpha+M_PI/6.0))*((l*cos(delta)/h - 1.0)*sin(alpha-M_PI/6.0) - cos(alpha));
    double T_S = S/D*(l*cos(delta)/h - 1.0);
    double T_B = 1.0 / (6.0*D) * (3.0*l*l*sin(delta)*cos(delta) + 6*b*h*cos(alpha-M_PI/3.0) - 6*l*h*sin(delta) - 2.0*sqrt(3.0)*b*l*cos(delta)*sin(alpha)) / (sqrt(3.0)*h*cos(alpha+M_PI/6.0));
    double T_D = 1.0;
    
    // Handle discontinuity at alpha=M_PI/3
    if (std::abs(alpha - M_PI/3.0) < 0.001) {
        h = l * cos(delta)/2.0;
        S = sqrt(h*h + b*b/3.0 + l*l*sin(delta)*sin(delta)
               - 2.0/sqrt(3.0)*l*b*sin(delta)*cos(alpha - M_PI/6.0) );
        V = sqrt(b*b + l*l - 2.0*l*b*sin(delta)*sin(alpha + M_PI/6.0));
        D = sqrt(h*h + b*b/3.0 + l*l - 2.0/sqrt(3.0)*l*b*sin(delta)*sin(alpha) - 2.0*l*h*cos(delta));
        
        T_V = V/D * (3*l/(2*b)*sin(delta) - 1.0);
        T_S = 1;
        T_B = 1.0/(6.0*D) * (2.0*b*b - 9.0*l*b*sin(delta) + 9.0*l*l*sin(delta)*sin(delta))/b;
        T_D = 1;
    }
    
    
    
    // Tension scaling factor:
    double P = 3000.0; // Pretension coefficient
    double T_scaling = P / (sqrt(T_S*T_S + T_V*T_V + T_D*T_D + T_B*T_B));
    //double T_scaling = 2000.0;
    T_S *= T_scaling;
    T_V *= T_scaling;
    T_D *= T_scaling;
    T_B *= T_scaling;
    
    //if (globalTime > 2){ // delay start of cable actuation
        std::cout << "(alpha, delta) = (" << alpha/M_PI*180.0 << ", " <<
                        delta/M_PI*180.0 << ") deg." << std::endl;
        //std::cout << "Lengths: " << S << ", " << V << ", " << D << ", " << b << std::endl;
        std::cout << "Tensions: " << T_S << ", " << T_V << ", " << T_D << ", " << T_B << std::endl;
        
        // Saddle
	    for (std::size_t i = 0; i < 6; i++) this->setRestLengthWithCmdTensionLength(i, T_S, S, dt);
	    // Verticle
	    for (std::size_t i = 6; i < 12; i++) this->setRestLengthWithCmdTensionLength(i, T_V, V, dt);
	    // Diagonal
	    for (std::size_t i = 12; i < 18; i++) this->setRestLengthWithCmdTensionLength(i, T_D, D, dt);
	    // Boundary/Base
	    for (std::size_t i = 18; i < 24; i++) this->setRestLengthWithCmdTensionLength(i, T_B, b, dt);
	    
    //}
}

// Command rest length for the specified actuator
void LengthController::setRestLengthWithCmdTensionLength(int actuatorID, double tension_cmd, double length_cmd, double dt)
{
    const tgSpringCable* m_springCable = actuators[actuatorID]->getSpringCable();
    // Rest length control:
    double commandRestLen = length_cmd - tension_cmd/(m_springCable->getCoefK());

    if (commandRestLen < 0) commandRestLen = 0;
    m_controllers[actuatorID]->control(dt, commandRestLen);
    actuators[actuatorID]->moveMotors(dt);
}
