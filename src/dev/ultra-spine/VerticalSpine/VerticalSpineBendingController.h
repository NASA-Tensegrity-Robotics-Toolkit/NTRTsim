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

#ifndef VERTICAL_SPINE_BENDING_CONTROLLER_H
#define VERTICAL_SPINE_BENDING_CONTROLLER_H

/**
 * @file VerticalSpineBendingController.h
 * @brief Contains the definition of class VerticalSpineBendingController.
 * @author Drew Sabelhaus, Brian Tietz, Michael Fanton
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "core/tgSubject.h"

// the data collection class
#include "sensors/tgDataObserver.h"
#include <string>

// Forward declarations
class VerticalSpineModel;

/**
 * A controller to apply uniform rest length offset to all cables in a
 * SpineModel. Does a one-time adjust of all cable rest lengths, thereby
 * tightening up the structure and keeping it held together without
 * further control input.
 */
class VerticalSpineBendingController : public tgObserver<VerticalSpineModel>, public tgSubject <VerticalSpineBendingController>
{
public:
	
  /**
   * Construct a VerticalSpineRestLengthController.
   * @param[in] restLengthDiff, the amount of cable retraction to enact.
   * This length will be subtracted from the geometric length of
   * each cable in the structure.
   */
  
  // Note that currently this is calibrated for decimeters.
  VerticalSpineBendingController();
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~VerticalSpineBendingController() { }

  /**
   * Apply the Bending controller. On setup, adjust the cable
   * lengths one time.
   * @param[in] subject - the SpineModel that is being controlled. Must
   * have a list of allMuscles populated
   */
  virtual void onSetup(VerticalSpineModel& subject);
    
  /**
   * The onStep method is not used for this controller.
   * @param[in] subject - the SpineModel that is being controlled. Must
   * have a list of allMuscles populated
   * @param[in] dt, current timestep must be positive
   */
  virtual void onStep(VerticalSpineModel& subject, double dt);
    
private:
	
  /**
   * The rest length adjustment to make to the cables. Set
   * in the constructor.
   */
    
   double verticalRLA1;
   double verticalRLA2;
   double verticalRLA3;
   double verticalRLA4;
   double verticalRLB1;
   double verticalRLB2;
   double verticalRLB3;
   double verticalRLB4;
   double dL;
   double state;
   double updateTime;

  // For data logging. TO-DO: implement this fully.
  tgDataObserver m_dataObserver;
  double m_updateTime;

};

#endif // Spine_BENDING_CONTROLLER_H
