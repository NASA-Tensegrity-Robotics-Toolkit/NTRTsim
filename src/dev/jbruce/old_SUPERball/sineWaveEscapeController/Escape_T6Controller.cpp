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
 * @file Escape_T6Controller.cpp
 * @brief Escape Controller for T6 
 * @author Steven Lessard
 * @version 1.0.0
 * $Id$
 */

// This module
#include "Escape_T6Controller.h"
// This application
#include "Escape_T6Model.h"
// This library
#include "core/tgBasicActuator.h"
// For AnnealEvolution
#include "learning/Configuration/configuration.h"
// The C++ Standard Library
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <string>

# define M_PI 3.14159265358979323846 
                               
using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
//Currently calibrated to decimeters
Escape_T6Controller::Escape_T6Controller(const double initialLength) :
    m_initialLengths(initialLength),
    m_totalTime(0.0),
    maxStringLengthFactor(0.50),
    nClusters(8),
    musclesPerCluster(3)
{
    clusters.resize(nClusters);
    for (int i=0; i<nClusters; i++) {
        clusters[i].resize(musclesPerCluster);
    }
}

/** Set the lengths of the muscles and initialize the learning adapter */
void Escape_T6Controller::onSetup(Escape_T6Model& subject)
{
    double dt = 0.0001;

    //Set the initial length of every muscle in the subject
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
        tgBasicActuator * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
        pMuscle->setControlInput(this->m_initialLengths, dt);
    }

    populateClusters(subject);
    initPosition = subject.getBallCOM();
    setupAdapter();
    initializeSineWaves(); // For muscle actuation

    vector<double> state; // For config file usage (including Monte Carlo simulations)

    //get the actions (between 0 and 1) from evolution (todo)
    actions = evolutionAdapter.step(dt,state);
 
    //transform them to the size of the structure
    actions = transformActions(actions);

    //apply these actions to the appropriate muscles according to the sensor values
    applyActions(subject,actions);
}

void Escape_T6Controller::onStep(Escape_T6Model& subject, double dt)
{
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    }
    m_totalTime+=dt;

    setPreferredMuscleLengths(subject, dt);
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    
    //Move motors for all the muscles
    for (size_t i = 0; i < muscles.size(); ++i)
    {
        tgBasicActuator * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
        pMuscle->moveMotors(dt);
    }

    //instead, generate it here for now!
    for(int i=0; i<muscles.size(); i++)
    {
        vector<double> tmp;
        for(int j=0;j<2;j++)
        {
            tmp.push_back(0.5);
        }
        actions.push_back(tmp);
    }
}

// So far, only score used for eventual fitness calculation of an Escape Model
// is the maximum distance from the origin reached during that subject's episode
void Escape_T6Controller::onTeardown(Escape_T6Model& subject) {
    std::vector<double> scores; //scores[0] == displacement, scores[1] == energySpent
    double distance = displacement(subject);
    double energySpent = totalEnergySpent(subject);

    //Invariant: For now, scores must be of size 2 (as required by endEpisode())
    scores.push_back(distance);
    scores.push_back(energySpent);

    std::cout << "Tearing down" << std::endl;
    evolutionAdapter.endEpisode(scores);

    // If any of subject's dynamic objects need to be freed, this is the place to do so
}

/** 
 * Returns the modified actions 2D vector such that 
 *   each action value is now scaled to fit the model
 * Invariant: actions[x].size() == 4 for all legal values of x
 * Invariant: Each actions[] contains: amplitude, angularFrequency, phaseChange, dcOffset
 */
vector< vector <double> > Escape_T6Controller::transformActions(vector< vector <double> > actions)
{
    bool usingManualParams = true;
    vector <double> manualParams(4 * nClusters, 1); // '4' for the number of sine wave parameters
    if (usingManualParams) { 
        std::cout << "Using manually set parameters\n"; 
        //string filename = "logs/trial_7/bestParametersNoOutliersSorted.dat";
        string filename = "logs/trial_8/bestParamsSorted.dat";
        int lineNumber = 1;
        manualParams = readManualParams(lineNumber, filename);
    } 

    double pretension = 0.90; // Tweak this value if need be
    // Minimum amplitude, angularFrequency, phaseChange, and dcOffset
    double mins[4]  = {m_initialLengths * (pretension - maxStringLengthFactor), 
                       0.3, //Hz
                       -1 * M_PI, 
                       m_initialLengths};// * (1 - maxStringLengthFactor)};

    // Maximum amplitude, angularFrequency, phaseChange, and dcOffset
    double maxes[4] = {m_initialLengths * (pretension + maxStringLengthFactor), 
                       20, //Hz (can cheat to 50Hz, if feeling immoral)
                       M_PI, 
                       m_initialLengths};// * (1 + maxStringLengthFactor)}; 
    double ranges[4] = {maxes[0]-mins[0], maxes[1]-mins[1], maxes[2]-mins[2], maxes[3]-mins[3]};

    for(int i=0;i<actions.size();i++) { //8x
        for (int j=0; j<actions[i].size(); j++) { //4x
            if (usingManualParams) {
                actions[i][j] = manualParams[i*actions[i].size() + j]*(ranges[j])+mins[j];
            } else {
                actions[i][j] = actions[i][j]*(ranges[j])+mins[j];
            }
        }
    }
    return actions;
}

/**
 * Defines each cluster's sine wave according to actions
 */
void Escape_T6Controller::applyActions(Escape_T6Model& subject, vector< vector <double> > actions)
{
    assert(actions.size() == clusters.size());

    // Apply actions by cluster
    for (size_t cluster = 0; cluster < clusters.size(); cluster++) {
        amplitude[cluster] = actions[cluster][0];
        angularFrequency[cluster] = actions[cluster][1];
        phaseChange[cluster] = actions[cluster][2];
        dcOffset[cluster] = actions[cluster][3];
    }
    //printSineParams();
}

void Escape_T6Controller::setupAdapter() {
    string suffix = "_Escape";
    string configAnnealEvolution = "Config.ini";
    AnnealEvolution* evo = new AnnealEvolution(suffix, configAnnealEvolution);
    bool isLearning = true;
    configuration configEvolutionAdapter;
    configEvolutionAdapter.readFile(configAnnealEvolution);

    evolutionAdapter.initialize(evo, isLearning, configEvolutionAdapter);
}

//TODO: Doesn't seem to correctly calculate energy spent by tensegrity
double Escape_T6Controller::totalEnergySpent(Escape_T6Model& subject) {
    double totalEnergySpent=0;

    vector<tgBasicActuator* > tmpStrings = subject.getAllMuscles();
    for(int i=0; i<tmpStrings.size(); i++)
    {
        tgBaseString::BaseStringHistory stringHist = tmpStrings[i]->getHistory();

        for(int j=1; j<stringHist.tensionHistory.size(); j++)
        {
            const double previousTension = stringHist.tensionHistory[j-1];
            const double previousLength = stringHist.restLengths[j-1];
            const double currentLength = stringHist.restLengths[j];
            //TODO: examine this assumption - free spinning motor may require more power         
            double motorSpeed = (currentLength-previousLength);
            if(motorSpeed > 0) // Vestigial code
                motorSpeed = 0;
            const double workDone = previousTension * motorSpeed;
            totalEnergySpent += workDone;
        }
    }
    return totalEnergySpent;
}

// Pre-condition: every element in muscles must be defined
// Post-condition: every muscle will have a new target length
void Escape_T6Controller::setPreferredMuscleLengths(Escape_T6Model& subject, double dt) {
    double phase = 0; // Phase of cluster1

    for(int cluster=0; cluster<nClusters; cluster++) {
        for(int node=0; node<musclesPerCluster; node++) {
            tgBasicActuator *const pMuscle = clusters[cluster][node];
            assert(pMuscle != NULL);
            double newLength = amplitude[cluster] * sin(angularFrequency[cluster] * m_totalTime + phase) + dcOffset[cluster];
            double minLength = m_initialLengths * (1-maxStringLengthFactor);
            double maxLength = m_initialLengths * (1+maxStringLengthFactor);
            if (newLength <= minLength) {
                newLength = minLength;
            } else if (newLength >= maxLength) {
                newLength = maxLength;
            }
            pMuscle->setControlInput(newLength, dt);
        }
        phase += phaseChange[cluster];
    }
}

void Escape_T6Controller::populateClusters(Escape_T6Model& subject) {
    for(int cluster=0; cluster < nClusters; cluster++) {
        ostringstream ss;
        ss << (cluster + 1);
        string suffix = ss.str();
        std::vector <tgBasicActuator*> musclesInThisCluster = subject.find<tgBasicActuator>("muscle cluster" + suffix);
        clusters[cluster] = std::vector<tgBasicActuator*>(musclesInThisCluster);
    }
}

void Escape_T6Controller::initializeSineWaves() {
    amplitude = new double[nClusters];
    angularFrequency = new double[nClusters];
    phaseChange = new double[nClusters]; // Does not use last value stored in array
    dcOffset = new double[nClusters];    
}

double Escape_T6Controller::displacement(Escape_T6Model& subject) {
    vector<double> finalPosition = subject.getBallCOM();

    // 'X' and 'Z' are irrelevant. Both variables measure lateral direction
    //assert(finalPosition[0] > 0); //Negative y-value indicates a flaw in the simulator that run (tensegrity went 'underground')

    const double newX = finalPosition[0];
    const double newZ = finalPosition[2];
    const double oldX = initPosition[0];
    const double oldZ = initPosition[2];

    const double distanceMoved = sqrt((newX-oldX) * (newX-oldX) + 
                                      (newZ-oldZ) * (newZ-oldZ));
    return distanceMoved;
}
                                         
std::vector<double> Escape_T6Controller::readManualParams(int lineNumber, string filename) {
    assert(lineNumber > 0);
    vector<double> result(32, 1.0);
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

void Escape_T6Controller::printSineParams() {
    for (size_t cluster = 0; cluster < clusters.size(); cluster++) {
        std::cout << "amplitude[" << cluster << "]: " << amplitude[cluster] << std::endl;
        std::cout << "angularFrequency[" << cluster << "]: " << angularFrequency[cluster] << std::endl;
        std::cout << "phaseChange[" << cluster << "]: " << phaseChange[cluster] << std::endl;
        std::cout << "dcOffset[" << cluster << "]: " << dcOffset[cluster] << std::endl;
    }    
}

