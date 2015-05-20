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
 * @file DuCTTLearnCtrl.cpp
 * @brief Learning Controller for DuCTT
 * @author Alexander Xydes
 * @version 1.0.0
 * $Id$
 */

// This module
#include "DuCTTLearnCtrl.h"

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
DuCTTLearnCtrl::DuCTTLearnCtrl(const double initialLength,
                                int axis,
                                double freq,
                                bool neuro,
                                string resourcePath,
                                string suffix,
                                string evoConfigFilename,
                                bool useManualParams,
                                string manualParamFile
) :
    DuCTTLearning(initialLength, axis, neuro, resourcePath, suffix, evoConfigFilename, useManualParams, manualParamFile),
    angularFrequency(freq),
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
    m_bTopPaused(false)
{
    prisms.resize(nPrisms);
    clusters.resize(nClusters);
    for (int i=0; i<nClusters; i++)
    {
        clusters[i].resize(musclesPerCluster);
    }
}

void DuCTTLearnCtrl::initBeforeAdapter(DuCTTRobotModel& subject) {
    for(int cluster=0; cluster < nClusters; cluster++) {
        ostringstream ss;
        ss << (cluster + 1);
        string suffix = ss.str();
        std::vector <tgBasicActuator*> musclesInThisCluster = subject.find<tgBasicActuator>("string cluster" + suffix);
        clusters[cluster] = std::vector<tgBasicActuator*>(musclesInThisCluster);
    }
    for(int prism=0; prism < nPrisms; prism++) {
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

void DuCTTLearnCtrl::initAfterAdapter(DuCTTRobotModel &subject) {
    amplitude = new double[nActions];
    phaseChange = new double[nActions]; // Does not use last value stored in array
    dcOffset = new double[nActions];
}

void DuCTTLearnCtrl::stepBeforeStart(DuCTTRobotModel& subject, double dt)
{
    if (isLocked(subject, false))
    {
        subject.getBottomPrismatic()->setPreferredLength(subject.getBottomPrismatic()->getActualLength());
    }
    return;
}

void DuCTTLearnCtrl::stepBeforeMove(DuCTTRobotModel& subject, double dt)
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

    setPreferredMuscleLengths(subject, dt);
    setPrismaticLengths(subject, dt);
}

void DuCTTLearnCtrl::teardownEnd(DuCTTRobotModel& subject)
{
    // If any of subject's dynamic objects need to be freed, this is the place to do so
    delete amplitude;
    delete phaseChange;
    delete dcOffset;
}

double DuCTTLearnCtrl::getFirstScore(DuCTTRobotModel& subject)
{
    return getCoIS(subject);
}

double DuCTTLearnCtrl::getSecondScore(DuCTTRobotModel& subject)
{
    return displacement(subject);
}


/** 
 * Returns the modified actions 2D vector such that 
 *   each action value is now scaled to fit the model
 * Invariant: actions[x].size() == 4 for all legal values of x
 * Invariant: Each actions[] contains: amplitude, angularFrequency, phaseChange, dcOffset
 */
vector< vector <double> > DuCTTLearnCtrl::transformActions(vector< vector <double> > actions)
{
    vector <double> params(N_PARAMS * nActions + 2, 1); // '4' for the number of sine wave parameters
    params = actions[0];

    //frequency (Hz)
    double minFreq = 0.01;
    double maxFreq = 20;
    double freqRange = maxFreq-minFreq;
    int freqOffset = 0;
    angularFrequency = params[freqOffset]*freqRange + minFreq;
    std::cerr << "Angular freq: " << angularFrequency << std::endl;

    //use touch sensors (bool)
    double touchParam;
    int touchOffset = 1;//actions[0].size() - (nActions*N_PARAMS);
    touchParam = (params[touchOffset]);
    m_bIgnoreTouchSensors = touchParam < 0.5;
    std::cerr << "Ignoring touch sensors: " << m_bIgnoreTouchSensors << std::endl;

    //historisis length (s)
    double minHistorisis = 0.0;
    double maxHistorisis = 2.0;
    double historisisRange = maxHistorisis - minHistorisis;
    int histOffset = 2;//params.size() - (nActions*N_PARAMS + 1);
    m_dHistorisisSeconds = params[histOffset]*historisisRange + minHistorisis;
    std::cerr << "Histeresis seconds: " << m_dHistorisisSeconds << std::endl;

    // Minimum amplitude, phaseChange, and dcOffset
    double mins[N_PARAMS]  = {
                               0,
                               -1 * M_PI, //phase change
                               0
                             };

    // Maximum amplitude, phaseChange, and dcOffset
    double maxes[N_PARAMS] = {40, //m_initialLengths * (pretension + maxStringLengthFactor), //amplitude
                               M_PI, //phase change
                               40
                             };
    double ranges[N_PARAMS] = {maxes[0]-mins[0], maxes[1]-mins[1], maxes[2]-mins[2]};

    std::vector< std::vector<double> > newActions (nActions);

    //going from 1x40 to 10x4
    //going from 1x16 to 4x4
    //#1=vertical, #2=saddle, 3-4=prism
    for(int i=0;i<nActions;i++) //10x
    {
        newActions[i] = std::vector<double>(N_PARAMS);
        for (int j=0; j<N_PARAMS; j++)  //4x
        {
            newActions[i][j] = params[3 + i*N_PARAMS + j]*(ranges[j])+mins[j];
        }
    }

    //going from 1x28 to 7x4
    //first 4 are vertical cables, #5 is all saddle cables, last two are prisms

    return newActions;
}

/**
 * Defines each cluster's sine wave according to actions
 */
void DuCTTLearnCtrl::applyActions(DuCTTRobotModel& subject, vector< vector <double> > actions)
{
    assert(actions.size() == nActions);

    // Apply actions by cluster
    for (size_t cluster = 0; cluster < clusters.size(); cluster++) {
        amplitude[cluster] = actions[cluster][0];
        phaseChange[cluster] = actions[cluster][2];
        dcOffset[cluster] = actions[cluster][3];
    }
    // Apply prism actions
    for (size_t prism = 0; prism < prisms.size(); prism++) {
        size_t idx = prism + (clusters.size()-1);
        amplitude[idx] = actions[idx][0];
        phaseChange[idx] = actions[idx][2];
        dcOffset[idx] = actions[idx][3];
    }
    //printSineParams();
}

// Pre-condition: every element in muscles must be defined
// Post-condition: every muscle will have a new target length
void DuCTTLearnCtrl::setPreferredMuscleLengths(DuCTTRobotModel& subject, double dt) {
    const double minLength = 1.2;//m_initialLengths * (1-maxStringLengthFactor);
    const double maxLength = 10;//m_initialLengths * (1+maxStringLengthFactor);

    for(int cluster=0; cluster<nClusters; cluster++) {
        for(int node=0; node<musclesPerCluster; node++) {
            tgBasicActuator *const pMuscle = clusters[cluster][node];
            assert(pMuscle != NULL);

            double newVelocity = amplitude[cluster] * sin(angularFrequency * m_totalTime + phaseChange[cluster]) + dcOffset[cluster];
//            if (newVelocity <= minLength) {
//                newVelocity = minLength;
//            } else if (newVelocity >= maxLength) {
//                newVelocity = maxLength;
//            }
            imp_controller->control(*pMuscle, dt, m_initialLength, newVelocity);
        }
    }
}

// Pre-condition: every element in muscles must be defined
// Post-condition: every muscle will have a new target length
//TODO: saturation of sin wave by touch sensors
//TODO: 'locking' of prismatics?
void DuCTTLearnCtrl::setPrismaticLengths(DuCTTRobotModel& subject, double dt) {

    for(int prism=0; prism<nPrisms; prism++) {
        size_t idx = prism + clusters.size()-1;
        tgPrismatic* const pPrism = prisms[prism];
        bool isTop = (pPrism == subject.getTopPrismatic());

        if (m_bIgnoreTouchSensors || !isLocked(subject, isTop))
        {
            double newLength = amplitude[idx] * sin(angularFrequency * m_totalTime + phaseChange[idx]) + dcOffset[idx];
            pPrism->setPreferredLength(newLength);
//            std::cerr << "Prism is top: " << isTop << ", newLength: " << newLength;
//            fprintf(stderr, ", amp: %f, freq: %f, phase: %f, offset: %f\n", amplitude[idx], angularFrequency[idx], phase, dcOffset[idx]);
        }
        else
        {
        }

    }
}

bool DuCTTLearnCtrl::isLocked(DuCTTRobotModel& subject, bool isTop)
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

bool DuCTTLearnCtrl::shouldPause(std::vector<tgTouchSensorModel*> touchSensors)
{
    bool shouldPause = true;

    for (size_t i=0; i<touchSensors.size(); i++)
    {
        if (!touchSensors[i]->isTouching()) shouldPause = false;
    }

    return shouldPause;
}

void DuCTTLearnCtrl::printSineParams() {
    for (size_t idx = 0; idx < nActions; idx++) {
        std::cout << "amplitude[" << idx << "]: " << amplitude[idx] << std::endl;
        std::cout << "angularFrequency[" << idx << "]: " << angularFrequency << std::endl;
        std::cout << "phaseChange[" << idx << "]: " << phaseChange[idx] << std::endl;
        std::cout << "dcOffset[" << idx << "]: " << dcOffset[idx] << std::endl;
    }
}
