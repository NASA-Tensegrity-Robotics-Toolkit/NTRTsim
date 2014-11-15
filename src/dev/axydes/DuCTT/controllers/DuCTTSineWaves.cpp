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
#include "../robot/DuCTTRobotModel.h"
#include "../robot/tgPrismatic.h"
#include "../robot/tgTouchSensorSphereModel.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

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

void DuCTTSineWaves::applySineWave(tgPrismatic* prism, bool shouldPause, bool shouldUnPause, double dt)
{
    cycle = sin(simTime * cpgFrequency + 2 * bodyWaves * M_PI + phaseOffsets[0]);
    target = offsetSpeed + cycle*cpgAmplitude;

    if (!shouldPause && shouldUnPause)
    {
        prism->setPreferredLength(target);
        std::cerr << "MOVING\n";
    }
    else
    {
        std::cerr << "PAUSING\n";
    }
}

bool DuCTTSineWaves::shouldPause(std::vector<tgTouchSensorSphereModel*> touchSensors)
{
    bool shouldPause = true;

    for (size_t i=0; i<touchSensors.size(); i++)
    {
        if (!touchSensors[i]->isTouching()) shouldPause = false;
    }

    return shouldPause;
}

void DuCTTSineWaves::onStep(DuCTTRobotModel& subject, double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        simTime += dt;
//        applySineWave(subject.getBottomPrismatic(), shouldPause(subject.bottomTouchSensors), !shouldPause(subject.topTouchSensors), dt);
        applySineWave(subject.getTopPrismatic(), shouldPause(subject.topTouchSensors), !shouldPause(subject.bottomTouchSensors), dt);
    }
}
