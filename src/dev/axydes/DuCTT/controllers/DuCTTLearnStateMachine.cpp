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
 * @file DuCTTLearnStateMachine.cpp
 * @brief Learning Controller for DuCTT
 * @author Alexander Xydes
 * @version 1.0.0
 * $Id$
 */

// This module
#include "DuCTTLearnStateMachine.h"

// This application
#include "../robot/DuCTTRobotModel.h"
#include "../robot/tgPrismatic.h"
#include "../robot/tgTouchSensorModel.h"

// This library
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"

#include "helpers/FileHelpers.h"

#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/NeuroEvolution/NeuroEvolution.h"
#include "learning/Configuration/configuration.h"

// The C++ Standard Library
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <string>

#define N_PARAMS 3
                               
using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
//Currently calibrated to decimeters
DuCTTLearnStateMachine::DuCTTLearnStateMachine(const double initialLength,
                                int axis,
                                bool neuro,
                                string resourcePath,
                                string suffix,
                                string evoConfigFilename
) :
    DuCTTLearning(initialLength, axis, neuro, resourcePath, suffix, evoConfigFilename),
//    nClusters(8),
    nClusters(2),
//    musclesPerCluster(1),
    musclesPerCluster(4),
    nPrisms(2),
    nActions(nClusters+nPrisms),
    m_bIgnoreTouchSensors(true),
    m_bRecordedStart(false),
    bottomCounter(0),
    topCounter(0),
    m_dHistorisisSeconds(0.5),
    m_bBottomPaused(false),
    m_bTopPaused(false),
    state(EXPAND_BOTTOM)
{
    prisms.resize(nPrisms);
    clusters.resize(nClusters);
    for (int i=0; i<nClusters; i++)
    {
        clusters[i].resize(musclesPerCluster);
    }
}

void DuCTTLearnStateMachine::initBeforeAdapter(DuCTTRobotModel& subject) {
    state = EXPAND_BOTTOM;
    for(int cluster=0; cluster < nClusters; cluster++)
    {
        ostringstream ss;
        ss << (cluster + 1);
        string suffix = ss.str();
        std::vector <tgBasicActuator*> musclesInThisCluster = subject.find<tgBasicActuator>("string cluster" + suffix);
        clusters[cluster] = std::vector<tgBasicActuator*>(musclesInThisCluster);
    }

    for(int prism=0; prism < nPrisms; prism++)
    {
        switch(prism)
        {
        case 0:
            prisms[prism] = subject.getBottomPrismatic();
            break;
        case 1:
            prisms[prism] = subject.getTopPrismatic();
            break;
        default:
            std::cerr << "ERROR: Too many prismatic joints!" << std::endl;
            break;
        }
    }
}

void DuCTTLearnStateMachine::initAfterAdapter(DuCTTRobotModel &subject) {
}

void DuCTTLearnStateMachine::stepBeforeStart(DuCTTRobotModel& subject, double dt)
{
}

void DuCTTLearnStateMachine::stepBeforeMove(DuCTTRobotModel& subject, double dt)
{
    btVector3 bottomCOM = subject.getTetraCOM();
    btVector3 topCOM = subject.getTetraCOM(false);

    double diffx = topCOM.x() - bottomCOM.x();
    double diffy = topCOM.y() - bottomCOM.y();
    double diffz = topCOM.z() - bottomCOM.z();

    m_bTilting = false;
    if (m_axis == 1)
    {
        double totalDiff = diffx + diffz;

        //starting to tilt?
        if (fabs(totalDiff) > 1)
        {
            m_bTilting = true;
        }
    }

//    std::cerr << "State: " << state << std::endl;
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

bool DuCTTLearnStateMachine::movePrism(tgPrismatic* prism, std::vector<tgTouchSensorModel*> sensors, double goal, double dt)
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
    bool pause = false;
    static int pauseCount = 0;
    pauseCount += shouldPause(sensors);
    if (pauseCount > m_dHistorisisSeconds*1000)
    {
        pauseCount = 0;
        pause = true;
    }

    return fabs(delta) < 0.1 || pause;
}

bool DuCTTLearnStateMachine::moveStrings(const std::vector<tgBasicActuator*> stringList, double goals, double dt)
{
    bool switchState = false;
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
        double setTension = imp_controller->control(*(stringList[i]), dt, goals);
        double currLength = stringList[i]->getCurrentLength();

        double delta = fabs(currLength - goals);
//        std::cerr << "String " << i << " length: " << currLength << ", delta: " << delta << std::endl;
        if (delta < stringLengthEPS) switchState = true;
    }
    return switchState;
}

void DuCTTLearnStateMachine::teardownEnd(DuCTTRobotModel& subject)
{
    // If any of subject's dynamic objects need to be freed, this is the place to do so
    state = EXPAND_BOTTOM;
}

/** 
 * Returns the modified actions 2D vector such that 
 *   each action value is now scaled to fit the model
 * Invariant: actions[x].size() == 4 for all legal values of x
 * Invariant: Each actions[] contains: amplitude, angularFrequency, phaseChange, dcOffset
 */
vector< vector <double> > DuCTTLearnStateMachine::transformActions(vector< vector <double> > actions)
{
    vector <double> params(N_PARAMS * nActions + 2, 1); // '4' for the number of sine wave parameters
    params = actions[0];

    //use touch sensors (bool)
    double touchParam;
    int touchOffset = 0;
    touchParam = (params[touchOffset]);
    m_bIgnoreTouchSensors = touchParam < 0.5;

    //historisis length (s)
    double minHistorisis = 0.0;
    double maxHistorisis = 2.0;
    double historisisRange = maxHistorisis - minHistorisis;
    int histOffset = touchOffset+1;
    m_dHistorisisSeconds = params[histOffset]*historisisRange + minHistorisis;

    //min string goal length (cm)
    double minMinLength = 0.0;
    double maxMinLength = 10.0;
    double minLengthRange = maxMinLength - minMinLength;
    int minLenOffset = histOffset+1;
    minStringLength = params[minLenOffset]*minLengthRange + minMinLength;

    //max string goal length (cm)
    double minMaxLength = 5.0;
    double maxMaxLength = 20.0;
    double maxLengthRange = maxMaxLength - minMaxLength;
    int maxLenOffset = minLenOffset+1;
    maxStringLength = params[maxLenOffset]*maxLengthRange + minMaxLength;

    //string goal length EPS (cm)
    double minLengthEPS = 0.0;
    double maxLengthEPS = 5.0;
    double lengthEPSRange = maxLengthEPS - minLengthEPS;
    int lenEPSOffset = maxLenOffset+1;
    stringLengthEPS = params[lenEPSOffset]*lengthEPSRange + minLengthEPS;

    return actions;
}

/**
 * Defines each cluster's sine wave according to actions
 */
void DuCTTLearnStateMachine::applyActions(DuCTTRobotModel& subject, vector< vector <double> > actions)
{
}

bool DuCTTLearnStateMachine::isLocked(DuCTTRobotModel& subject, bool isTop)
{
    static const double maxCount = m_dHistorisisSeconds*1000;//1000Hz timestep

    bool sPause = false;
    bool sUnPause = false;
    if (isTop)
    {
//        sPause = shouldPause(subject.topTouchSensors) && !shouldPause(subject.bottomTouchSensors);
        sPause = shouldPause(subject.topTouchSensors);// && !shouldPause(subject.bottomTouchSensors);
        sUnPause = shouldPause(subject.bottomTouchSensors);
//        if (sPause)
        if ((sPause && !m_bTopPaused) || (sUnPause && m_bTopPaused))
            topCounter++;
    }
    else
    {
//        sPause = shouldPause(subject.bottomTouchSensors) && !shouldPause(subject.topTouchSensors);
        sPause = shouldPause(subject.bottomTouchSensors);// && !shouldPause(subject.topTouchSensors);
        sUnPause = shouldPause(subject.topTouchSensors);
//        if (sPause)
        if ((sPause && !m_bBottomPaused) || (sUnPause && m_bBottomPaused))
            bottomCounter++;
    }
    bool isLocked = false;
    if (sPause)
    {
        if (isTop && topCounter > maxCount)
        {
            if (!m_bTopPaused)
            {
                isLocked = true;
                m_bTopPaused = true;
            }
            else
            {
                isLocked = false;
                m_bTopPaused = false;
            }
            topCounter = 0;
        }
        else if (!isTop && bottomCounter > maxCount)
        {
            if (!m_bBottomPaused)
            {
                isLocked = true;
                m_bBottomPaused = true;
            }
            else
            {
                isLocked = false;
                m_bBottomPaused = false;
            }
            bottomCounter = 0;
        }
    }

    return isLocked;
}

bool DuCTTLearnStateMachine::shouldPause(std::vector<tgTouchSensorModel*> touchSensors)
{
    bool shouldPause = true;

    for (size_t i=0; i<touchSensors.size(); i++)
    {
        if (!touchSensors[i]->isTouching()) shouldPause = false;
    }

    return shouldPause;
}

void DuCTTLearnStateMachine::printSineParams() {
}
