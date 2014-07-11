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
 * @file T6PrefLengthController.cpp
 * @brief Preferred Length Controller for T6. Constant speed motors are used in muscles to reach preffered length
 * @author Atil Iscen
 * @version 1.0.0
 * $Id$
 */

// This module
#include "T6PrefLengthController.h"
// This application
#include "T6Model.h"
// This library
#include "core/tgLinearString.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

//Constructor using the model subject and a single pref length for all muscles.
T6PrefLengthController::T6PrefLengthController(const double initialLength)
{
	this->m_initialLengths=initialLength;
}

//Fetch all the muscles and set their preffered length
void T6PrefLengthController::onSetup(T6Model& subject)
{
	const std::vector<tgLinearString*> muscles = subject.getAllMuscles();
	for (size_t i = 0; i < muscles.size(); ++i)
	{
		tgLinearString * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->setRestLength(this->m_initialLengths,0.0001);
	}
}

void T6PrefLengthController::onStep(T6Model& subject, double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
	const std::vector<tgLinearString*> muscles = subject.getAllMuscles();
	for (size_t i = 0; i < muscles.size(); ++i)
	{
		tgLinearString * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->moveMotors(dt);
	}

}
