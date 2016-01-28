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

#ifndef FLEX_CONTROLLER_H
#define FLEX_CONTROLLER_H

/**
 * @file FlexController.h
 * @brief Controller to simulate flexion of the knee model (ThirdTFModel)
 * @author Dennis Castro
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "core/tgSubject.h"
// The C++ Standard Library
#include "sensors/tgDataObserver.h"
#include <string>

// Forward declarations
class TFModel;

/**
 * A class that constructs the TFModel
 */
class FlexController : public tgObserver<TFModel>, public tgSubject<FlexController>
{
public:

    //** A class with virtual member functions must have a virtual destructor. */
    FlexController();


    virtual ~FlexController() {}
    /**
     * Notify the observers when a step action has occurred.
     * @param[in,out] subject the subject being observed
     * @param[in] the number of seconds since the previous call; must be
     * positive
     */
    virtual void onStep(TFModel& subject, double dt);
    
    /**
     * Notify the observers when an attach action has occurred.
     * Will only occur once, typically before setup
     * @param[in,out] subject the subject being observed
     */
    //virtual void onAttach(TFModel& subject) { }
    
    /**
     * Notify the observers when a setup action has occurred.
     * @param[in,out] subject the subject being observed
     */
    virtual void onSetup(TFModel& subject);

    /**
     * Notify the observers when a teardown action has occurred.
     * @param[in,out] subject the subject being observed
     */    
    //virtual void onTeardown(TFModel& subject);

private:
    //time functions of muscle lengths
    double verticalBFA1;
   double verticalBFA2;
   double verticalGSA3;
   double verticalGSA4;
   double verticalBFB1;
   double verticalBFB2;
   double verticalGSB3;
   double verticalGSB4;
   double dL;
   double state;
   double updateTime;

  // For data logging. TO-DO: implement this fully.
  //tgDataObserver m_dataObserver;
  double m_updateTime;
};

#endif  // FLEX_CONTROLLER.H
