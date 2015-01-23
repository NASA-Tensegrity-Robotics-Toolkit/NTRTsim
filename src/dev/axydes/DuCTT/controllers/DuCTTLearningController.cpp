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
 * @file DuCTTLearningController.cpp
 * @brief Learning Controller for DuCTT
 * @author Alexander Xydes
 * @version 1.0.0
 * $Id$
 */

// This module
#include "DuCTTLearningController.h"

// This application
#include "../robot/DuCTTRobotModel.h"
#include "../robot/tgPrismatic.h"
#include "../robot/tgTouchSensorSphereModel.h"

// This library
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"

#include "helpers/FileHelpers.h"

#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Configuration/configuration.h"

// The C++ Standard Library
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <string>

#define M_PI 3.14159265358979323846
#define N_PARAMS 4
                               
using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
//Currently calibrated to decimeters
DuCTTLearningController::DuCTTLearningController(const double initialLength,
                                                const bool useManualParams,
                                                const string manParamFile,
                                                string resourcePath,
                                                string suffix,
                                                string evoConfigFilename
                                                 ) :
    m_evoConfigFilename(evoConfigFilename),
    m_evolution(suffix, evoConfigFilename),
    m_isLearning(false),
    m_initialLengths(initialLength),
    m_usingManualParams(useManualParams),
    m_manualParamFile(manParamFile),
    m_totalTime(0.0),
    maxStringLengthFactor(1.50),
    nClusters(8),
    musclesPerCluster(1),
    nPrisms(2),
    nActions(nClusters+nPrisms),
    imp_controller(new tgImpedanceController(0.01, 500, 10)),
    m_bBadRun(false),
    m_bIgnoreTouchSensors(false),
    m_bRecordedStart(false)
{
    std::string path;
    if (resourcePath != "")
    {
        path = FileHelpers::getResourcePath(resourcePath);
    }
    else
    {
        path = "";
    }
    m_evoConfig.readFile(path+m_evoConfigFilename);
    m_isLearning = m_evoConfig.getBoolValue("learning");

    prisms.resize(nPrisms);
    clusters.resize(nClusters);
    for (int i=0; i<nClusters; i++)
    {
        clusters[i].resize(musclesPerCluster);
    }
}

/** Set the lengths of the muscles and initialize the learning adapter */
void DuCTTLearningController::onSetup(DuCTTRobotModel& subject)
{
    double dt = 0.0001;

    //Set the initial length of every muscle in the subject
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
        tgBasicActuator * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
        pMuscle->setControlInput(this->m_initialLengths, dt);
    }

    //Set the initial lengths of the prismatic joints
    subject.getBottomPrismatic()->setPreferredLength(subject.getBottomPrismatic()->getMaxLength());
    subject.getBottomPrismatic()->moveMotors(dt);
    subject.getTopPrismatic()->setPreferredLength(subject.getTopPrismatic()->getMinLength());
    subject.getTopPrismatic()->moveMotors(dt);

    populateClusters(subject);
    m_evolutionAdapter.initialize(&m_evolution, m_isLearning, m_evoConfig);
    initializeSineWaves(); // For muscle actuation

    /* Empty vector signifying no state information
     * All parameters are stateless parameters, so we can get away with
     * only doing this once
     */
    vector<double> state;

    //get the actions (between 0 and 1) from evolution
    m_actions = m_evolutionAdapter.step(dt,state);
 
    //transform them to the size of the structure
    m_actions = transformActions(m_actions);

    //apply these actions to the appropriate muscles according to the sensor values
    applyActions(subject,m_actions);
}

void DuCTTLearningController::onStep(DuCTTRobotModel& subject, double dt)
{
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    }
    m_totalTime+=dt;

    if (m_totalTime < 5) return;
    else if (!m_bRecordedStart)
    {
        initPosition = subject.getCOM();
        m_bRecordedStart = true;
    }

    setPreferredMuscleLengths(subject, dt);
    setPrismaticLengths(subject, dt);

    moveMotors(subject, dt);

    //TODO: check for bad run?

    /** What is this?
    for(int i=0; i<nActions; i++)
    {
        vector<double> tmp;
        for(int j=0;j<2;j++)
        {
            tmp.push_back(0.5);
        }
        m_actions.push_back(tmp);
    }
    /**/
}

void DuCTTLearningController::moveMotors(DuCTTRobotModel &subject, double dt)
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

// So far, only score used for eventual fitness calculation of an Escape Model
// is the maximum distance from the origin reached during that subject's episode
void DuCTTLearningController::onTeardown(DuCTTRobotModel& subject) {
    std::vector<double> scores; //scores[0] == displacement, scores[1] == energySpent
    double distance = displacement(subject);
    double energySpent = totalEnergySpent(subject);

    //Invariant: For now, scores must be of size 2 (as required by endEpisode())
    if (!m_bBadRun)
    {
        scores.push_back(distance);
    }
    else
    {
        scores.push_back(-1);
    }

    scores.push_back(energySpent);

    std::cout << "Tearing down" << std::endl;
    m_evolutionAdapter.endEpisode(scores);

    // If any of subject's dynamic objects need to be freed, this is the place to do so
    delete amplitude;
    delete angularFrequency;
    delete phaseChange;
    delete dcOffset;

    m_totalTime = 0.0;
    m_bRecordedStart = false;
    m_bBadRun = false;
}

/** 
 * Returns the modified actions 2D vector such that 
 *   each action value is now scaled to fit the model
 * Invariant: actions[x].size() == 4 for all legal values of x
 * Invariant: Each actions[] contains: amplitude, angularFrequency, phaseChange, dcOffset
 */
vector< vector <double> > DuCTTLearningController::transformActions(vector< vector <double> > actions)
{
    vector <double> manualParams(N_PARAMS * nActions, 1); // '4' for the number of sine wave parameters
    if (m_usingManualParams) {
        std::cout << "Using manually set parameters\n"; 
        int lineNumber = 1;
        manualParams = readManualParams(lineNumber, m_manualParamFile);
    } 

    double pretension = 0.90; // Tweak this value if need be
    // Minimum amplitude, angularFrequency, phaseChange, and dcOffset
    double mins[N_PARAMS]  = {1.2, //m_initialLengths * (pretension - maxStringLengthFactor),
                       0.3, //Hz
                       -1 * M_PI, 
                       1.2}; //m_initialLengths};// * (1 - maxStringLengthFactor)};

    // Maximum amplitude, angularFrequency, phaseChange, and dcOffset
    double maxes[N_PARAMS] = {10, //m_initialLengths * (pretension + maxStringLengthFactor),
                       20, //Hz (can cheat to 50Hz, if feeling immoral)
                       M_PI, 
                       10};//m_initialLengths};// * (1 + maxStringLengthFactor)};
    double ranges[N_PARAMS] = {maxes[0]-mins[0], maxes[1]-mins[1], maxes[2]-mins[2], maxes[3]-mins[3]};

    std::vector< std::vector<double> > newActions (nActions);
    //going from 1x40 to 10x4
    for(int i=0;i<nActions;i++) //10x
    {
        newActions[i] = std::vector<double>(N_PARAMS);
        for (int j=0; j<N_PARAMS; j++)  //4x
        {
            if (m_usingManualParams)
            {
                newActions[i][j] = manualParams[i*N_PARAMS + j]*(ranges[j])+mins[j];
            }
            else
            {
                newActions[i][j] = actions[0][(i*N_PARAMS)+j]*(ranges[j])+mins[j];
            }
        }
    }

    m_bIgnoreTouchSensors = (actions[0][actions[0].size()-1]);
    std::cerr << "Ignoring touch sensors: " << m_bIgnoreTouchSensors << std::endl;

    return newActions;
}

/**
 * Defines each cluster's sine wave according to actions
 */
void DuCTTLearningController::applyActions(DuCTTRobotModel& subject, vector< vector <double> > actions)
{
    assert(actions.size() == nActions);

    // Apply actions by cluster
    for (size_t cluster = 0; cluster < clusters.size(); cluster++) {
        amplitude[cluster] = actions[cluster][0];
        angularFrequency[cluster] = actions[cluster][1];
        phaseChange[cluster] = actions[cluster][2];
        dcOffset[cluster] = actions[cluster][3];
    }
    // Apply prism actions
    for (size_t prism = 0; prism < prisms.size(); prism++) {
        size_t idx = prism + clusters.size()-1;
        amplitude[idx] = actions[idx][0];
        angularFrequency[idx] = actions[idx][1];
        phaseChange[idx] = actions[idx][2];
        dcOffset[idx] = actions[idx][3];
    }
    //printSineParams();
}

//TODO: Doesn't seem to correctly calculate energy spent by tensegrity
//TODO: punish slack strings
//TODO: too much pretension
//TODO: historisis effect of latching onto wall (need to come away from wall
//      a certain amount to be unlocked
double DuCTTLearningController::totalEnergySpent(DuCTTRobotModel& subject) {
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

// Pre-condition: every element in muscles must be defined
// Post-condition: every muscle will have a new target length
void DuCTTLearningController::setPreferredMuscleLengths(DuCTTRobotModel& subject, double dt) {
    double phase = 0; // Phase of cluster1
    const double minLength = 1.2;//m_initialLengths * (1-maxStringLengthFactor);
    const double maxLength = 10;//m_initialLengths * (1+maxStringLengthFactor);

    for(int cluster=0; cluster<nClusters; cluster++) {
        for(int node=0; node<musclesPerCluster; node++) {
            tgBasicActuator *const pMuscle = clusters[cluster][node];
            assert(pMuscle != NULL);
            double newLength = amplitude[cluster] * sin(angularFrequency[cluster] * m_totalTime + phase) + dcOffset[cluster];
            if (newLength <= minLength) {
                newLength = minLength;
            } else if (newLength >= maxLength) {
                newLength = maxLength;
            }
            imp_controller->control(*pMuscle, dt, m_initialLengths, newLength);
        }
        phase += phaseChange[cluster];
    }
}

// Pre-condition: every element in muscles must be defined
// Post-condition: every muscle will have a new target length
//TODO: saturation of sin wave by touch sensors
//TODO: 'locking' of prismatics?
void DuCTTLearningController::setPrismaticLengths(DuCTTRobotModel& subject, double dt) {
    double phase = 0; // Phase of cluster1
    const double minLength = subject.getBottomPrismatic()->getMinLength();
    const double maxLength = subject.getBottomPrismatic()->getMaxLength();

    for(int prism=0; prism<nPrisms; prism++) {
        size_t idx = prism + clusters.size()-1;
        tgPrismatic* const pPrism = prisms[prism];
        bool isTop = (pPrism == subject.getTopPrismatic());

        if (m_bIgnoreTouchSensors || !isLocked(subject, isTop))
        {
            double newLength = amplitude[idx] * sin(angularFrequency[idx] * m_totalTime + phase) + dcOffset[idx];
            if (newLength <= minLength) {
                newLength = minLength;
            } else if (newLength >= maxLength) {
                newLength = maxLength;
            }

            pPrism->setPreferredLength(newLength);
        }
        else
        {
        }

        phase += phaseChange[idx];
    }
}

bool DuCTTLearningController::isLocked(DuCTTRobotModel& subject, bool isTop)
{
    bool isLocked = false;
    if (isTop)
    {
        isLocked = shouldPause(subject.topTouchSensors) && !shouldPause(subject.bottomTouchSensors);
    }
    else
    {
        isLocked = shouldPause(subject.bottomTouchSensors) && !shouldPause(subject.topTouchSensors);
    }
    return isLocked;
}

bool DuCTTLearningController::shouldPause(std::vector<tgTouchSensorSphereModel*> touchSensors)
{
    bool shouldPause = true;

    for (size_t i=0; i<touchSensors.size(); i++)
    {
        if (!touchSensors[i]->isTouching()) shouldPause = false;
    }

    return shouldPause;
}

void DuCTTLearningController::populateClusters(DuCTTRobotModel& subject) {
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

void DuCTTLearningController::initializeSineWaves() {
    amplitude = new double[nActions];
    angularFrequency = new double[nActions];
    phaseChange = new double[nActions]; // Does not use last value stored in array
    dcOffset = new double[nActions];
}

double DuCTTLearningController::displacement(DuCTTRobotModel& subject) {
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
//    return distanceMoved;
//    return newY - oldY;
    return newZ - oldZ;
}
                                         
std::vector<double> DuCTTLearningController::readManualParams(int lineNumber, string filename) {
    assert(lineNumber > 0);
    vector<double> result(nActions*4, 1.0);
    string line;
    ifstream infile(filename.c_str(), ifstream::in);

    // Grab line from input file
    if (infile.is_open()) {
        for (int i=0; i<lineNumber; i++) {
            getline(infile, line);
        }
        infile.close();
    }

    //cout << "Using: " << line << " as input for starting parameter values\n";

    // Split line into parameters
    stringstream lineStream(line);
    string cell;
    int iCell = 0;
    while(getline(lineStream,cell,',')) {
        result[iCell] = atof(cell.c_str());
        iCell++;
    }

    // Tweak each read-in parameter by as much as 0.5% (params range: [0,1])
    for (int i=0; i < result.size(); i++) {
        //std::cout<<"Cell " << i << ": " << result[i] << "\n";
        double seed = ((double) (rand() % 100)) / 100;
        result[i] += (0.01 * seed) - 0.005; // Value +/- 0.005 of original
        //std::cout<<"Cell " << i << ": " << result[i] << "\n";
    }

    return result;
}

void DuCTTLearningController::printSineParams() {
    for (size_t idx = 0; idx < nActions; idx++) {
        std::cout << "amplitude[" << idx << "]: " << amplitude[idx] << std::endl;
        std::cout << "angularFrequency[" << idx << "]: " << angularFrequency[idx] << std::endl;
        std::cout << "phaseChange[" << idx << "]: " << phaseChange[idx] << std::endl;
        std::cout << "dcOffset[" << idx << "]: " << dcOffset[idx] << std::endl;
    }
}
