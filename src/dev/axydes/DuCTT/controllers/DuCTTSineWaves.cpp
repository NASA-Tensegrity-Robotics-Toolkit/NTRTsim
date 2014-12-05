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

#include "core/Muscle2P.h"
#include "core/tgLinearString.h"
#include "core/ImpedanceControl.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

DuCTTSineWaves::DuCTTSineWaves(double targetDist) :
    in_controller(new ImpedanceControl(100, 500, 50)),
    out_controller(new ImpedanceControl(0.01, 500, 10)),
    insideLength(6.5),
    simTime(0.0),
    offsetLength(0.5),
    cpgAmplitude(14.5),
    cpgFrequency(1.0),
    bodyWaves(1.0),
    cycle(0.0),
    target(0.0),
    offsetLengthPrism(0.0),
    cpgAmplitudePrism(10.0),
    cpgFrequencyPrism(1.0),
    bodyWavesPrism(1.0),
    cyclePrism(0.0),
    targetPrism(0.0),
    targetDist(targetDist),
    recordedStartCOM(false),
    move(true)
{
    phaseOffsets.clear();
    phaseOffsets.push_back(0);
    phaseOffsets.push_back(M_PI/4);
    phaseOffsets.push_back(M_PI/2);
    phaseOffsets.push_back(3*M_PI/4);
    phaseOffsets.push_back(M_PI);
}

void DuCTTSineWaves::applySineWave(tgPrismatic* prism, bool shouldPause, bool shouldUnPause, double dt, int phase)
{
    cyclePrism = sin(simTime * cpgFrequencyPrism + 2 * bodyWavesPrism * M_PI + phaseOffsets[phase]);
    targetPrism = offsetLengthPrism + cyclePrism*cpgAmplitudePrism;

    double currLength = prism->getActualLength();
    double err = fabs(currLength - targetPrism);

    if (!shouldPause && shouldUnPause && err < 3.0)
    {
        prism->setPreferredLength(targetPrism);
    }
}

void DuCTTSineWaves::applyImpedanceControlInside(const std::vector<tgLinearString*> stringList, double dt)
{
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
        double setTension = in_controller->control(stringList[i],
                                            dt,
                                            insideLength
                                            );
        #if (0) // Conditional compile for verbose control
        std::cout << "Inside String " << i << " tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
        #endif
    }
}

void DuCTTSineWaves::applyImpedanceControlOutside(const std::vector<tgLinearString*> stringList,
                                                            double dt,
                                                            std::size_t phase)
{
    cycle = sin(simTime * cpgFrequency + 2 * bodyWaves * M_PI + phaseOffsets[phase]);
    target = offsetLength + cycle*cpgAmplitude;

    if (target < 0) target *= -1;

    for(std::size_t i = 0; i < stringList.size(); i++)
    {
        double setTension = out_controller->control(stringList[i], dt, target);
//        stringList[i]->setRestLength(target,dt);
//        double setTension = stringList[i]->getMuscle()->getTension();
        #if(0) // Conditional compile for verbose control
        std::cout << "Outside String " << i << ", target: " << target << " com tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
        #endif
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

        if (simTime < 5 || !move) return;
        else if (!recordedStartCOM)
        {
            startCOM = subject.getCOM();
            recordedStartCOM = true;
        }

        btVector3 com = subject.getCOM();
        double dist = startCOM.distance(com);

        if (targetDist < 0 || dist < targetDist)
        {
            applyImpedanceControlOutside(subject.getAllMuscles(), dt, 1);
            applySineWave(subject.getBottomPrismatic(), shouldPause(subject.bottomTouchSensors), !shouldPause(subject.topTouchSensors), dt, 4);
//            applySineWave(subject.getBottomPrismatic(), false, true, dt, 4);
    //        applySineWave(subject.getTopPrismatic(), shouldPause(subject.topTouchSensors), !shouldPause(subject.bottomTouchSensors), dt);
        }
        else if (dist >= targetDist)
        {
            std::cerr << "Total Dist moved: " << dist << std::endl;
            move = false;
        }
    }
}
