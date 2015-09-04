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

#ifndef UPPERLIMBTRIANGLE_CONTROLLER_H
#define UPPERLIMBTRIANGLE_CONTROLLER_H

/**
 * @file UpperLimbTriangleController.h
 * @brief Contains the definition of class UpperLimbTriangleController
 * @author Steven Lessard
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "learning/Adapters/AnnealAdapter.h"
#include <vector>

// Forward declarations
class UpperLimbTriangleModel;

//namespace std for vectors
using namespace std;

/**
 * Preferred Length Controller for UpperLimbTriangleModel. This controllers sets a preferred rest length for the muscles.
 * Constant speed motors are used in muscles to move the rest length to the preffered length over time.
 * The assumption here is that motors are constant speed independent of the tension of the muscles.
 * motorspeed and movemotors are defined at the tgBasicActuator class.
 */
class UpperLimbTriangleController : public tgObserver<UpperLimbTriangleModel>
{
public:
	
  /**
   * Construct a UpperLimbTriangleController with the initial preferred length.
   *
   */
  
  // Note that currently this is calibrated for decimeters.
	UpperLimbTriangleController(const double prefLength, double timestep);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~UpperLimbTriangleController() { }

  virtual void onSetup(UpperLimbTriangleModel& subject);
    
  virtual void onStep(UpperLimbTriangleModel& subject, double dt);

protected:

  virtual vector< vector <double> > transformActions(vector< vector <double> > act);

  virtual void applyActions (UpperLimbTriangleModel& subject, vector< vector <double> > act);

private:
  double m_initialLengths;
  double m_totalTime;
  double dt;
  AnnealAdapter evolutionAdapter;
    
  void setBrachioradialisTargetLength(UpperLimbTriangleModel& subject, double dt);
  void setAnconeusTargetLength(UpperLimbTriangleModel& subject, double dt);
  void moveAllMotors(UpperLimbTriangleModel& subject, double dt);
  void updateActions(UpperLimbTriangleModel& subject, double dt);
};

#endif // UPPERLIMBTRIANGLE_CONTROLLER_H
