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
 * @file DuCTTSimple.cpp
 * @brief Contains the implementation of the class DuCTTSimple
 * @author Alexander Xydes
 * $Id$
 */

// This module
#include "DuCTTSimple.h"

// Its subject
#include "../robot/DuCTTRobotModel.h"
#include "../robot/tgPrismatic.h"
#include "../robot/tgTouchSensorSphereModel.h"

#include "core/abstractMarker.h"
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

DuCTTSimple::DuCTTSimple(double targetDist) :
    in_controller(new tgImpedanceController(100, 500, 50)),
    out_controller(new tgImpedanceController(0.01, 500, 10)),
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
    move(true),
    shouldBotPause(true),
    shouldTopPause(false),
    state(EXPAND_BOTTOM)
{
    phaseOffsets.clear();
    phaseOffsets.push_back(0);
    phaseOffsets.push_back(M_PI/4);
    phaseOffsets.push_back(M_PI/2);
    phaseOffsets.push_back(3*M_PI/4);
    phaseOffsets.push_back(M_PI);
}

void DuCTTSimple::applySineWave(tgPrismatic* prism, bool shouldPause, bool shouldUnPause, double dt, int phase)
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

void DuCTTSimple::applyImpedanceControlInside(const std::vector<tgBasicActuator*> stringList, double dt)
{
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
        double setTension = in_controller->control(*(stringList[i]),
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

void DuCTTSimple::applyImpedanceControlOutside(const std::vector<tgBasicActuator*> stringList,
                                                            double dt,
                                                            std::size_t phase)
{
    cycle = sin(simTime * cpgFrequency + 2 * bodyWaves * M_PI + phaseOffsets[phase]);
    target = offsetLength + cycle*cpgAmplitude;

    for(std::size_t i = 0; i < stringList.size(); i++)
    {
//        if (stringList[i]->hasTag("cluster3"))
//            target = insideLength;
        double setTension = out_controller->control(*(stringList[i]), dt, target);
//        stringList[i]->setRestLength(target,dt);
//        double setTension = stringList[i]->getMuscle()->getTension();
        #if(0) // Conditional compile for verbose control
        std::cout << "Outside String " << i << ", target: " << target << " com tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
        #endif
    }
}

bool DuCTTSimple::shouldPause(std::vector<tgTouchSensorSphereModel*> touchSensors)
{
    bool shouldPause = true;

    for (size_t i=0; i<touchSensors.size(); i++)
    {
        if (!touchSensors[i]->isTouching()) shouldPause = false;
    }

    return shouldPause;
}

bool DuCTTSimple::checkPause(std::vector<tgTouchSensorSphereModel*> touchSensors, bool top)
{
    bool shouldPause = true;

    for (size_t i=0; i<touchSensors.size(); i++)
    {
        if (!touchSensors[i]->isTouching()) shouldPause = false;
    }

    if (top)
    {
        shouldTopPause = shouldPause;
    }
    else
    {
        shouldBotPause = shouldPause;
    }
    return true;
}

bool DuCTTSimple::movePrism(tgPrismatic* prism, std::vector<tgTouchSensorSphereModel*> sensors, double goal, double dt)
{
    double currPos = prism->getActualLength();
    double delta = (goal-currPos);
//    std::cerr << "delta: " << delta << std::endl;
//    std::cerr << "curr goal: " << (currPos+delta) << std::endl;

    double dist = (prism->getMaxVelocity() / dt);

//    prism->setPreferredLength(currPos + (delta / dt));
    if (delta > 0)
        prism->setPreferredLength(currPos + dist);
    if (delta < 0)
        prism->setPreferredLength(currPos - dist);

//    prism->setPreferredLength(goal);
//    prism->moveMotors(dt);

//    return shouldPause(sensors);
    return fabs(delta) < 0.1;
}

bool DuCTTSimple::moveStrings(const std::vector<tgBasicActuator*> stringList, double goals, double dt)
{
    bool switchState = false;
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
        double setTension = out_controller->control(*(stringList[i]), dt, goals);
        double currLength = stringList[i]->getCurrentLength();

        double delta = fabs(currLength - goals);
        std::cerr << "String " << i << " length: " << currLength << ", delta: " << delta << std::endl;
        if (delta < 2) switchState = true;
    }
    return switchState;
}

void DuCTTSimple::onStep(DuCTTRobotModel& subject, double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        std::cerr << "State: " << state << std::endl;
        double maxStringLength = 10;
        double minStringLength = 4;
        switch(state)
        {
        case EXPAND_BOTTOM:
            if (movePrism(subject.getBottomPrismatic(), subject.bottomTouchSensors, subject.getBottomPrismatic()->getMaxLength(), dt))
                state = RETRACT_TOP;
            break;
        case RETRACT_TOP:
            if (movePrism(subject.getTopPrismatic(), subject.topTouchSensors, subject.getTopPrismatic()->getMinLength(), dt))
                state = PUSH_TOP;
            break;
        case PUSH_TOP:
            (moveStrings(subject.getSaddleMuscles(), minStringLength, dt));
            if (moveStrings(subject.getVertMuscles(), maxStringLength, dt))
                state = EXPAND_TOP;
            break;
        case EXPAND_TOP:
            if (movePrism(subject.getTopPrismatic(), subject.topTouchSensors, subject.getTopPrismatic()->getMaxLength(), dt))
                state = RETRACT_BOTTOM;
            break;
        case RETRACT_BOTTOM:
            if (movePrism(subject.getBottomPrismatic(), subject.bottomTouchSensors, subject.getBottomPrismatic()->getMinLength(), dt))
                state = PULL_BOTTOM;
            break;
        case PULL_BOTTOM:
            (moveStrings(subject.getSaddleMuscles(), maxStringLength, dt));
            if (moveStrings(subject.getVertMuscles(), minStringLength, dt))
                state = EXPAND_BOTTOM;
            break;
        default:
            break;
        }
    }
}

void DuCTTSimple::onSetup(DuCTTRobotModel& subject)
{
    state = EXPAND_BOTTOM;
    simTime = 0;
    recordedStartCOM = false;
}

void DuCTTSimple::onTeardown(DuCTTRobotModel& subject)
{
    state = EXPAND_BOTTOM;
    simTime = 0;
    recordedStartCOM = false;
}
