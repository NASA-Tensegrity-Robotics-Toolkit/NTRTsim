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

#ifndef UPPERLIMB_CONTROLLER_H
#define UPPERLIMB_CONTROLLER_H

/**
 * @file UpperLimbController.h
 * @brief Contains the definition of class UpperLimbController
 * @author Steven Lessard
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "learning/Adapters/AnnealAdapter.h"
#include <vector>

// Forward declarations
class UpperLimbModel;

//namespace std for vectors
using namespace std;

/**
 * Preferred Length Controller for UpperLimbModel. This controllers sets a preferred rest length for the muscles.
 * Constant speed motors are used in muscles to move the rest length to the preffered length over time.
 * The assumption here is that motors are constant speed independent of the tension of the muscles.
 * motorspeed and movemotors are defined at the tgBasicActuator class.
 */
class UpperLimbController : public tgObserver<UpperLimbModel>
{
public:
	
  /**
   * Construct a UpperLimbController with the initial preferred length.
   */
  
  // Note that currently this is calibrated for decimeters.
	UpperLimbController(const double prefLength, double timestep);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~UpperLimbController() { }
  virtual void onSetup(UpperLimbModel& subject);
  virtual void onStep(UpperLimbModel& subject, double dt);

protected:

  virtual vector< vector <double> > transformActions(vector< vector <double> > act);
  virtual void applyActions (UpperLimbModel& subject, vector< vector <double> > act);

private:
  double m_initialLengths;
  double m_totalTime;
  double dt;
  size_t nInputNeurons;
  size_t nHiddenNeurons; // One hidden layer
  size_t nOutputNeurons;
  AnnealAdapter evolutionAdapter;
    
  void initializeNeuralNet(UpperLimbModel& subject);
  void initializeMusclePretensions(UpperLimbModel& subject);
  void setBrachioradialisTargetLength(UpperLimbModel& subject, double dt);
  void setAnconeusTargetLength(UpperLimbModel& subject, double dt);
  void moveAllMotors(UpperLimbModel& subject, double dt);
  void updateActions(UpperLimbModel& subject, double dt);
};

#endif // UPPERLIMB_CONTROLLER_H
