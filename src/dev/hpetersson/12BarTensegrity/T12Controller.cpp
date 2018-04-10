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
 * @file T12Controller.cpp
 * @brief Controller for T12 Model 
 * @author Hannah Petersson based on code from Steven Lessard
 * @version 1.0.0
 * $Id$
 */

// This module
#include "T12Controller.h"
// This application
#include "T12Model.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "core/abstractMarker.h"
#include "core/tgObserver.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// For AnnealEvolution
#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Adapters/AnnealAdapter.h"
#include "learning/Configuration/configuration.h"
// File helpers to use resources folder
#include "helpers/FileHelpers.h"
// The C++ Standard Library
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <string>
#include <iostream> 
#include <fstream>

# define M_PI 3.14159265358979323846 
                               
using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
//Currently calibrated to decimeters
T12Controller::T12Controller(T12Model* subject, const double initialLength, double startTime) :
    m_initialLengths(initialLength),
    m_startTime(startTime),
    m_totalTime(0.0),
    maxStringLengthFactor(0.50),
    nClusters(6),  // 6 = number of squares on 12Bar. On SUPERball, the number of faces is 8.
    musclesPerCluster(4) // 4 = number of muscles per square. On SUPERball, the number is 3.
{
    clusters.resize(nClusters);
    for (int i=0; i<nClusters; i++) {
        clusters[i].resize(musclesPerCluster);
    }
}

/** Set the lengths of the muscles and initialize the learning adapter */
void T12Controller::onSetup(T12Model& subject)
{
    double dt = 0.0001;

    //Set the initial length of every muscle in the subject
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    //std::cout << "Muscle.size: " << muscles.size() << std::endl;
    for (size_t i = 0; i < muscles.size(); ++i) {
	//std::cout << "Muscles: " << muscles[i] << std::endl;
        tgBasicActuator * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
        pMuscle->setControlInput(this->m_initialLengths, dt);
    }

    //populateClusters(subject);
    initPosition = subject.getBallCOM();

    // DEBUGGING
    cout << "initPosition x: " << initPosition[1] << endl;
    cout << "initPosition y: " << initPosition[2] << endl;
    cout << "initPosition z: " << initPosition[3] << endl;

    //setupAdapter();
    initializeSineWaves(); // For muscle actuation

    vector<double> state(nClusters); // For config file usage (including Monte Carlo simulations)
    vector< vector <double> > actions(nClusters);

    //get the actions (between 0 and 1) from evolution (todo)
    //actions = evolutionAdapter.step(dt,state);
    
    //transform them to the size of the structure
    actions = transformActions(actions);

    cout << "actions.size(): " << actions.size() << endl;

    //apply these actions to the appropriate muscles according to the sensor values
    applyActions(subject,actions);
}

void T12Controller::onStep(T12Model& subject, double dt)
{
    double distanceMoved;

    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    }
    m_totalTime+=dt;

    if( m_totalTime > m_startTime) {
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
        
        distanceMoved = displacement(subject);
        cout << "Distance moved from initial position: " << distanceMoved << endl;
    }
}

// So far, only score used for eventual fitness calculation of an Escape Model
// is the maximum distance from the origin reached during that subject's episode
void T12Controller::onTeardown(T12Model& subject) {
    std::vector<double> scores; //scores[0] == displacement, scores[1] == energySpent
    double distance = displacement(subject);
    //double energySpent = totalEnergySpent(subject);

    //Invariant: For now, scores must be of size 2 (as required by endEpisode())
    scores.push_back(distance);
    //scores.push_back(energySpent);

    std::cout << "Tearing down" << std::endl;
    evolutionAdapter.endEpisode(scores);

    // If any of subject's dynamic objects need to be freed, this is the place to do so
}

/** 
 * Returns the modified actions 2D vector such that 
 * each action value is now scaled to fit the model
 * Invariant: actions[x].size() == 4 for all legal values of x
 * Invariant: Each actions[] contains: amplitude, angularFrequency, phaseChange, dcOffset
 */
vector< vector <double> > T12Controller::transformActions(vector< vector <double> > actions1D)
{
    vector< vector <double> > actions2D(nClusters, vector<double>(4)); // Vector to be returned
    cout << "actions2D size: " << actions2D.size() << ", " << actions2D[0].size() << endl;
    
    bool usingManualParams = true;
    vector <double> manualParams(24, 1); // '4' for the number of sine wave parameters, nClusters = 6 -> 24 total
    if (usingManualParams) {
        const char* filename = "logs/paramSortedBestTrials.dat";
        std::cout << "Using manually set parameters from file " << filename << endl; 
        int lineNumber = 1;
        manualParams = readManualParams(lineNumber, filename);  
        cout << "manualParams.size(): " << manualParams.size() << endl;
  } 

    double pretension = 0.9; // Tweak this value if need be

    cout << "maxStrinLengthFactor :" << maxStringLengthFactor << endl;

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

    cout << "Actions matrix is of size: (" << actions2D.size() << ", " << actions2D[0].size() << ")" << endl;

    for(int i=0;i<actions2D.size();i++) { //6x
        for (int j=0; j<actions2D[i].size(); j++) { //4x
            if (usingManualParams) {
                actions2D[i][j] = manualParams[i*actions2D[i].size() + j]*(ranges[j])+mins[j];
                cout << "action: " << actions2D[i][j] << endl;
            } else {
                actions[i][j] = actions[i][j]*(ranges[j])+mins[j];
                cout << "action: " << actions[i][j] << endl;
            }
        }
    }
    cout << "transformActions() completed." << endl;
    return actions2D;
}

/**
 * Defines each cluster's sine wave according to actions
 */
void T12Controller::applyActions(T12Model& subject, vector< vector <double> > actions)
{
    // DEBUGGING
    cout << "Action size: " << actions.size() << endl;

    assert(actions.size() == clusters.size());

    // DEBUGGING
    cout << "Cluster size: " << clusters.size() << endl;

    // Apply actions by cluster
    for (size_t cluster = 0; cluster < clusters.size(); cluster++) {
        amplitude[cluster] = actions[cluster][0];    
        angularFrequency[cluster] = actions[cluster][1];
        phaseChange[cluster] = actions[cluster][2];
        dcOffset[cluster] = actions[cluster][3];
    }
    printSineParams();
}


void T12Controller::setupAdapter() {
    string suffix = "_Escape";
    string configAnnealEvolution = "Config.ini";
    AnnealEvolution* evo = new AnnealEvolution(suffix, configAnnealEvolution);
    bool isLearning = true;
    configuration configEvolutionAdapter;
    configEvolutionAdapter.readFile(configAnnealEvolution);

    evolutionAdapter.initialize(evo, isLearning, configEvolutionAdapter);
}

/*
//TODO: Doesn't seem to correctly calculate energy spent by tensegrity
double T12Controller::totalEnergySpent(T12Model& subject) {
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
}*/

// Pre-condition: every element in muscles must be defined
// Post-condition: every muscle will have a new target length
void T12Controller::setPreferredMuscleLengths(T12Model& subject, double dt) {
    double phase = 0; // Phase of cluster1
  
    int nMuscles = 36;
    int oldCluster = 0;
    int cluster = 0;
    bool clusterMember = true; 	// Parameter to determine if action should be taken or not

    for( int iMuscle = 0; iMuscle < nMuscles; iMuscle ++) {

  //      cout << "Going through muscle: " << iMuscle << endl;
        oldCluster = cluster;
        clusterMember = true;

        const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles(); // Save all muscles in vector (could be done outside?)
        tgBasicActuator *const pMuscle = muscles[iMuscle];

        assert(pMuscle != NULL);

        // Group muscles in clusters
        if (iMuscle == 1 || iMuscle == 2|| iMuscle == 18|| iMuscle == 20) {
            cluster = 0;
        } else if (iMuscle == 3 || iMuscle == 5 || iMuscle == 23 || iMuscle == 34) {
            cluster = 1;
        } else if (iMuscle == 7 || iMuscle == 8 || iMuscle == 25 || iMuscle == 30) {
            cluster = 2;
        } else if (iMuscle == 9 || iMuscle == 11 || iMuscle == 28 || iMuscle == 29) {
            cluster = 3;
        } else if (iMuscle == 12 || iMuscle == 13 || iMuscle == 21 || iMuscle == 31) {
            cluster = 4;
        } else if (iMuscle == 15 || iMuscle == 16 || iMuscle == 26 || iMuscle == 32) {
            cluster = 5;
        } else {
            clusterMember = false;
        }

//        cout << "clusterMember: " << clusterMember << endl;

        if (clusterMember) {
            double newLength = amplitude[cluster] * sin(angularFrequency[cluster] * m_totalTime + phase) + dcOffset[cluster];
            double minLength = m_initialLengths * (1-maxStringLengthFactor);
            double maxLength = m_initialLengths * (1+maxStringLengthFactor);
        
            if (newLength <= minLength) {
                newLength = minLength;
            } else if (newLength >= maxLength) {
                newLength = maxLength;
            }
            pMuscle->setControlInput(newLength, dt);
            
            if (oldCluster != cluster) {
                phase += phaseChange[cluster];
            }
        }
    }
}

/*
void T12Controller::populateClusters(T12Model& subject) {
    for(int cluster=0; cluster < nClusters; cluster++) {
        ostringstream ss;
        ss << (cluster + 1);
        string suffix = ss.str();
        std::vector <tgBasicActuator*> musclesInThisCluster = subject.find<tgBasicActuator>("muscle cluster" + suffix);
        clusters[cluster] = std::vector<tgBasicActuator*>(musclesInThisCluster);
    
        // DEBUGGING
        cout << "Cluster: " << clusters[cluster] << endl;
    }
}
*/
/* Initializes sine waves, each cluster has identical parameters */
void T12Controller::initializeSineWaves() {
    amplitude = new double[nClusters];
    angularFrequency = new double[nClusters];
    phaseChange = new double[nClusters]; // Does not use last value stored in array
    dcOffset = new double[nClusters];    

    // DEBUGGING
    //cout << " amplitude: " << amplitude << " angularFrequency: " << angularFrequency << " phaseChange: " << phaseChange << " dcOffset: " << dcOffset << endl;
}

double T12Controller::displacement(T12Model& subject) {
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

std::vector<double> T12Controller::readManualParams(int lineNumber, const char* filename) {
    assert(lineNumber > 0);
    vector<double> result(32, 1.0); // Why 32?
    string line;
    ifstream infile(filename, ifstream::in);

    // Grab line from input file
    if (infile.is_open()) {
        cout << "File found and opened." << endl;
        for (int i=0; i<lineNumber; i++) {
            getline(infile, line); // Get line from infile and store in line
        }
        infile.close();
    }
    else {
        cout << "Error opening file." << endl;
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

    // Tweak each read-in parameter by as much as 0.5% (params range: [0,1])     <----- WHY?
    for (int i=0; i < result.size(); i++) {
        std::cout<<"Cell " << i << ": " << result[i] << "\n";
        double seed = ((double) (rand() % 100)) / 100;
        result[i] += (0.01 * seed) - 0.005; // Value +/- 0.005 of original

        if(result[i] >= 1) {
            result[i] = result[i] - 0.5; // Dummy solution before values from learning is found for 12 bar
        }
      
        std::cout<<"Cell " << i << ": " << result[i] << "\n";
    }

    cout << "readManualParams() finished. result.size = " << result.size() << endl;

    return result;
}

void T12Controller::printSineParams() {
    for (size_t cluster = 0; cluster < clusters.size(); cluster++) {
        std::cout << "amplitude[" << cluster << "]: " << amplitude[cluster] << std::endl;
        std::cout << "angularFrequency[" << cluster << "]: " << angularFrequency[cluster] << std::endl;
        std::cout << "phaseChange[" << cluster << "]: " << phaseChange[cluster] << std::endl;
        std::cout << "dcOffset[" << cluster << "]: " << dcOffset[cluster] << std::endl;
    }    
}


