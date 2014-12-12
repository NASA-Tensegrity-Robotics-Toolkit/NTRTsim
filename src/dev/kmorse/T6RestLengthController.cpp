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
 * @file T6RestLengthController.cpp
 * @brief Implementation of a rest length controller for T6Model.
 * @author Drew Sabelhaus and Brian Tietz
 * @version 1.0.0
 * $Id$
 */

// This module
#include "T6RestLengthController.h"
// This application
#include "T6Model.h"
// This library
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

T6RestLengthController::T6RestLengthController(const double restLengthDiff) :
    m_restLengthDiff(restLengthDiff) 
{
    if (restLengthDiff < 0.0)
    {
        throw std::invalid_argument("You tried to push a rope!");
    }
}

void T6RestLengthController::onSetup(T6Model& subject)
{
    // Do a one-time update of all cable rest lengths
    // First, get all muscles (cables)
    const std::vector<tgSpringCableActuator*> sca = subject.getAllMuscles();
    const std::vector<tgBasicActuator*> muscles = tgCast::filter<tgSpringCableActuator, tgBasicActuator> (sca);
    // then, iterate over all muscles
    for (size_t i = 0; i < muscles.size(); ++i)
    {
        tgBasicActuator * const pMuscle = muscles[i];
	assert(pMuscle != NULL);
   
    double desiredRestLength = pMuscle->getStartLength() - m_restLengthDiff;
    // Note that the single step version of setRestLength is used here,
    // since we only want to call it once (not iteratively like the original.)
    pMuscle->setRestLengthSingleStep(desiredRestLength);
    }
}

void T6RestLengthController::onStep(T6Model& subject, double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
      // Nothing!!
    }
}
