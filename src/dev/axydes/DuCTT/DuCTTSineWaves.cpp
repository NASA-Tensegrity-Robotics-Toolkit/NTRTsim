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
 * @file DuCTTSineWaves.cpp
 * @brief Contains the implementation of the class DuCTTSineWaves
 * @author Alexander Xydes
 * $Id$
 */

// This module
#include "DuCTTSineWaves.h"
// Its subject
#include "DuCTTRobotModel.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

#include "tgPrismatic.h"

DuCTTSineWaves::DuCTTSineWaves() :
    offsetSpeed(0.0),
    cpgAmplitude(10.0),
    cpgFrequency(1.00),
    bodyWaves(1.0),
    simTime(0.0),
    cycle(0.0),
    target(0.0)
{
    phaseOffsets.clear();
    phaseOffsets.push_back(M_PI/2);
    phaseOffsets.push_back(0);
    phaseOffsets.push_back(0);
}

void DuCTTSineWaves::applySineWave(std::vector<tgPrismatic*> prisms, double dt)
{
    simTime += dt;
    cycle = sin(simTime * cpgFrequency + 2 * bodyWaves * M_PI + phaseOffsets[0]);
    target = offsetSpeed + cycle*cpgAmplitude;
    for (std::size_t i=0; i<prisms.size(); i++)
    {
        prisms[i]->setPreferredLength(target);
    }
}

void DuCTTSineWaves::onStep(DuCTTRobotModel& subject, double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        applySineWave(subject.getAllPrismatics(), dt);
    }
}
