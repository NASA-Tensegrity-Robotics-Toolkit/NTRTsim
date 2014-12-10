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

#ifndef SUPERBALL_LEARNINGCONTROLLER_H
#define SUPERBALL_LEARNINGCONTROLLER_H

/**
 * @file SuperBallLearningController.h.h
 * @brief Contains the definition of class SuperBallLearningController.
 * @author Atil Iscen
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "learning/Adapters/AnnealAdapter.h"
#include <vector>
#include "BulletDynamics/Dynamics/btRigidBody.h"

// Forward declarations
class SuperBallModel;

//namespace std for vectors
using namespace std;

/**
 * Preferred Length Controller for T6. This controllers sets a preferred rest length for the muscles.
 * Constant speed motors are used in muscles to move the rest length to the preffered length over time.
 * The assumption here is that motors are constant speed independent of the tension of the muscles.
 * motorspeed and movemotors are defined at the tgBasicActuator class.
 */
class SuperBallPrefLengthController : public tgObserver<SuperBallModel>
{
public:
	
  /**
   * Construct a T6PrefLengthController with the initial preferred length.
   *
   */
  
  // Note that currently this is calibrated for decimeters.
	SuperBallPrefLengthController(const double prefLength=5);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~SuperBallPrefLengthController() { }

  virtual void onSetup(SuperBallModel& subject);
    
  virtual void onStep(SuperBallModel& subject, double dt);

  virtual void onTeardown(SuperBallModel& subject);

protected:

  vector< vector <double> > receiveActionsFromEvolution();

  virtual vector< vector <double> > transformActions(vector< vector <double> > act);

  virtual void applyActions (SuperBallModel& subject, vector< vector <double> > act, vector<double> state);

  double calculateDistanceMoved();

private:
  double m_initialLengths;
  double m_totalTime;
  btRigidBody *goalPoint;

  AnnealEvolution *evolution;
	
  vector<double> initialPosition;
  SuperBallModel *m_subject;
};

#endif // SUPERBALL_LEARNINGCONTROLLER_H
