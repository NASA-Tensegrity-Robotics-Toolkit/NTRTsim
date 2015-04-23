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
 * @file DuCTTLearning.cpp
 * @brief Learning Controller for DuCTT
 * @author Alexander Xydes
 * @version 1.0.0
 * $Id$
 */

// This module
#include "DuCTTLearning.h"

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

#define M_PI 3.14159265358979323846
#define N_PARAMS 3
                               
using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
//Currently calibrated to decimeters
DuCTTLearning::DuCTTLearning(const double initialLength,
                                int axis,
                                bool neuro,
                                string resourcePath,
                                string suffix,
                                string evoConfigFilename,
                                bool useManualParams,
                                string manualParamFile
                                 ) :
    m_evoConfigFilename(evoConfigFilename),
    m_evolution(suffix, evoConfigFilename, resourcePath),
    m_NeuroEvolution(suffix, evoConfigFilename, resourcePath),
    m_isLearning(false),
    m_initialLength(initialLength),
    m_axis(axis),
    m_bUseNeuro(neuro),
    m_totalTime(0.0),
    imp_controller(new tgImpedanceController(1000, 500, 10)),
    m_bBadRun(false),
    m_bRecordedStart(false),
    m_bUseManualParams(useManualParams),
    m_ManualParamFile(manualParamFile)
{
    if (resourcePath != "")
    {
        m_ResourcePath = FileHelpers::getResourcePath(resourcePath);
    }
    else
    {
        m_ResourcePath = "";
    }
    m_evoConfig.readFile(m_ResourcePath+m_evoConfigFilename);
    m_isLearning = m_evoConfig.getBoolValue("learning");
}

/** Set the lengths of the muscles and initialize the learning adapter */
void DuCTTLearning::onSetup(DuCTTRobotModel& subject)
{
    std::cout << getName() << ": Setting up" << std::endl;
    double dt = 0.0001;

    //Set the initial length of every muscle in the subject
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
        tgBasicActuator * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
        pMuscle->setControlInput(this->m_initialLength, dt);
    }

    //Set the initial lengths of the prismatic joints
    subject.getBottomPrismatic()->setPreferredLength(subject.getBottomPrismatic()->getMinLength());
    subject.getBottomPrismatic()->moveMotors(dt);
    subject.getTopPrismatic()->setPreferredLength(subject.getTopPrismatic()->getMinLength());
    subject.getTopPrismatic()->moveMotors(dt);

    initBeforeAdapter(subject);

    if (m_bUseNeuro)
    {
        m_neuroAdapter.initialize(&m_NeuroEvolution, m_isLearning, m_evoConfig);
    }
    else
    {
        m_evolutionAdapter.initialize(&m_evolution, m_isLearning, m_evoConfig);
    }

    initAfterAdapter(subject);

    /* Empty vector signifying no state information
     * All parameters are stateless parameters, so we can get away with
     * only doing this once
     */
    vector<double> state;

        //get the actions (between 0 and 1) from evolution
    if (m_bUseNeuro)
    {
        m_actions = m_neuroAdapter.step(dt,state);
    }
    else
    {
        m_actions = m_evolutionAdapter.step(dt,state);
    }
 
    //transform them to the size of the structure
    m_actions = transformActions(m_actions);

    //apply these actions to the appropriate muscles according to the sensor values
    applyActions(subject,m_actions);
}

void DuCTTLearning::onStep(DuCTTRobotModel& subject, double dt)
{
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    }
    m_totalTime+=dt;

    if (m_totalTime < 3)
    {
        stepBeforeStart(subject, dt);
        return;
    }
    else if (!m_bRecordedStart)
    {
        initPosition = subject.getCOM();
        m_bRecordedStart = true;
    }

    stepBeforeMove(subject, dt);

    moveMotors(subject, dt);

    stepAfterMove(subject, dt);
}

void DuCTTLearning::moveMotors(DuCTTRobotModel &subject, double dt)
{
    //Move motors for all the muscles
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i)
    {
        tgBasicActuator * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
        pMuscle->moveMotors(dt);
    }

    //Move prismatic joints
    subject.getBottomPrismatic()->moveMotors(dt);
    subject.getTopPrismatic()->moveMotors(dt);
}

void DuCTTLearning::onTeardown(DuCTTRobotModel& subject)
{
    std::vector<double> scores;

    //Invariant: For now, scores must be of size 2 (as required by endEpisode())
    if (!m_bBadRun)
    {
        scores.push_back(getFirstScore(subject));
    }
    else
    {
        scores.push_back(-1);
    }

    scores.push_back(getSecondScore(subject));

    if (m_bUseNeuro)
    {
        m_neuroAdapter.endEpisode(scores);
    }
    else
    {
        m_evolutionAdapter.endEpisode(scores);
    }

    // If any of subject's dynamic objects need to be freed, this is the place to do so
    teardownEnd(subject);

    m_totalTime = 0.0;
    m_bRecordedStart = false;
    m_bBadRun = false;
    std::cout << getName() << ": Torn down" << std::endl;
}

double DuCTTLearning::getFirstScore(DuCTTRobotModel& subject)
{
    return displacement(subject);
}

double DuCTTLearning::getSecondScore(DuCTTRobotModel& subject)
{
    return totalEnergySpent(subject);
}

//TODO: Doesn't seem to correctly calculate energy spent by tensegrity
//TODO: punish slack strings
//TODO: too much pretension
double DuCTTLearning::totalEnergySpent(DuCTTRobotModel& subject) {
    double totalEnergySpent=0;

    vector<tgBasicActuator* > tmpStrings = subject.getAllMuscles();
    for(int i=0; i<tmpStrings.size(); i++)
    {
        tgSpringCableActuator::SpringCableActuatorHistory stringHist = tmpStrings[i]->getHistory();

        for(int j=1; j<stringHist.tensionHistory.size(); j++)
        {
            const double previousTension = stringHist.tensionHistory[j-1];
            const double previousLength = stringHist.restLengths[j-1];
            const double currentLength = stringHist.restLengths[j];
            //TODO: examine this assumption - free spinning motor may require more power         
            double motorSpeed = (currentLength-previousLength);
            if(motorSpeed > 0) // Vestigial code
            {
                motorSpeed = 0;
            }
            else
            {
            }
            const double workDone = previousTension * motorSpeed;
            totalEnergySpent += workDone;
        }
    }
    return totalEnergySpent;
}

double DuCTTLearning::displacement(DuCTTRobotModel& subject) {
    btVector3 finalPosition = subject.getCOM();

    //assert(finalPosition[0] > 0);

    const double newX = finalPosition.x();
    const double newY = finalPosition.y();
    const double newZ = finalPosition.z();
    const double oldX = initPosition.x();
    const double oldY = initPosition.y();
    const double oldZ = initPosition.z();

    const double distanceMoved = sqrt(
                                      (newX-oldX) * (newX-oldX) +
                                      (newY-oldY) * (newY-oldY) +
                                      (newZ-oldZ) * (newZ-oldZ)
                                    );

    switch(m_axis)
    {
    case 0:
        return fabs(newX - oldX);
    case 2:
        return fabs(newZ - oldZ);
    case 3:
        return distanceMoved;
    case 1:
    default:
        return newY - oldY;
    }
}

std::vector<double> DuCTTLearning::readManualParams(int lineNumber)
{
    std::string filename = m_ResourcePath + m_ManualParamFile;
    std::cout << "Using manual parameters: " << filename << std::endl;
    assert(lineNumber > 0);
    std::vector<double> result(32, 1.0);
    std::string line;
    std::ifstream infile(filename.c_str(), std::ifstream::in);

    // Grab line from input file
    if (infile.is_open()) {
        std::cout << "OPENED FILE\n";
        for (int i=0; i<lineNumber; i++) {
            getline(infile, line);
        }
        infile.close();
    } else {
        std::cerr << "Error: Manual Parameters file not found\n";
        exit(1);
    }

    //cout << "Using: " << line << " as input for starting parameter values\n";

    // Split line into parameters
    std::stringstream lineStream(line);
    std::string cell;
    int iCell = 0;
    while(getline(lineStream,cell,',')) {
        result[iCell] = atof(cell.c_str());
        iCell++;
    }

    return result;
}
