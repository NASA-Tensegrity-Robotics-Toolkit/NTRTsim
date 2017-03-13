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
 * @file VerticalSpinePassiveController.cpp
 * @brief Implementation of a passive controller for VerticalSpineModel.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "VerticalSpinePassiveController.h"
// This application
#include "VerticalSpineModel.h"
// This library
//#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "sensors/tgDataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

#include "helpers/FileHelpers.h"

VerticalSpinePassiveController::VerticalSpinePassiveController():
  m_updateTime(0.01),
  m_dataObserver("logs/vertspine_passive_")
{
}

void VerticalSpinePassiveController::onSetup(VerticalSpineModel& subject){
  m_dataObserver.onSetup(subject);
  // Since this class only updates the logs every so often (according to
  // m_updateTime), need to track a counter of time between steps.
  updateTime = 0.0;

  // Debugging: what does this model look like?
  //std::cout << "This VerticalSpineModel has the following contents:" << std::endl;
  //std::cout << subject << std::endl;
}

void VerticalSpinePassiveController::onStep(VerticalSpineModel& subject, double dt)
{
  // first, check if our input arguments are sane
  if( dt <= 0.0)
  {
    throw std::invalid_argument("dt is not positive");
  }
  else
  {
    // check our update time. This is so that samples are taken only every
    // certain number of steps, not all the time.
    updateTime += dt;
    if( updateTime >= m_updateTime )
    {
      // Take a step: call the observers' step function.
      // TO FIX: why do we have to call the data logger's onStep individually?
      notifyStep(updateTime);
      m_dataObserver.onStep(subject, updateTime);
      // then, reset the counter to zero
      updateTime = 0.0;
    }
  } 
  // log only.
  //notifyStep(dt);
  //m_dataObserver.onStep(subject, dt);
}
