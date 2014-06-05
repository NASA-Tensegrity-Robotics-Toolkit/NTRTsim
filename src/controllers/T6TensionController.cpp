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

// This module
#include "T6TensionController.h"
// This application
#include "models/T6Model.h"
// This library
#include "core/tgLinearString.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

T6TensionController::T6TensionController(const double tension) :
    m_tension(tension) 
{
    if (tension < 0.0)
    {
        throw std::invalid_argument("Negative tension");
    }
}

void T6TensionController::onStep(T6Model& subject, double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        const std::vector<tgLinearString*> muscles = subject.getAllMuscles();
        for (size_t i = 0; i < muscles.size(); ++i)
        {
        tgLinearString * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
            pMuscle->tensionMinLengthController(m_tension, dt);
        }
    }
}
