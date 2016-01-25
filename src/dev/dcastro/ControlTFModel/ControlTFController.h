/*
 * Copyright © 2015, United States Government, as represented by the
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

#ifndef CONTROLTF_CONTROLLER_H
#define CONTROLTF_CONTROLLER_H

/**
 * @file ControlTFController.h
 * @brief Contains the definition of class ControlTFController
 * @author Dennis Castro
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "learning/Adapters/AnnealAdapter.h" //************What is this?***********8
#include <vector>

// Forward declarations
class ControlTFModel;

//namespace std for vectors
using namespace std;

/**
 * Preferred Length Controller for ControlTFModel. This controllers sets a preferred rest length for the muscles.
 * Constant speed motors are used in muscles to move the rest length to the preffered length over time.
 * The assumption here is that motors are constant speed independent of the tension of the muscles.
 * motorspeed and movemotors are defined at the tgBasicActuator class.
 */
class ControlTFController : public tgObserver<ControlTFModel>
{
public:
	
  /**
   * Construct a ControlTFController with the initial preferred length.
   *
   */
  
  // Note that currently this is calibrated for decimeters.
	ControlTFController(const double prefLength, double timestep);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~ControlTFController() { }

  virtual void onSetup(ControlTFModel& subject);
    
  virtual void onStep(ControlTFModel& subject, double dt);

protected:

  virtual vector< vector <double> > transformActions(vector< vector <double> > act);

  virtual void applyActions (ControlTFModel& subject, vector< vector <double> > act);

private:
  double m_initialLengths;
  double m_totalTime;
  double dt;
  //AnnealAdapter evolutionAdapter;
    
  void setGastroTargetLength(ControlTFModel& subject, double dt);
 // void setAnconeusTargetLength(ScarrArmModel& subject, double dt);
  void moveAllMotors(ControlTFModel& subject, double dt);
  void updateActions(ControlTFModel& subject, double dt);
};

#endif // CONTROLTF_CONTROLLER_H
