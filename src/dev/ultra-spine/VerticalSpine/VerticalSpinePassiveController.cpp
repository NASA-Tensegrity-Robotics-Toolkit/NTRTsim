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
  m_updateTime(0.0),
  m_dataObserver("logs/vertspine_1-2-3-4_")
{
}

void VerticalSpinePassiveController::onSetup(VerticalSpineModel& subject){
  m_dataObserver.onSetup(subject);
}

void VerticalSpinePassiveController::onStep(VerticalSpineModel& subject, double dt)
{
  // do nothing.
}
