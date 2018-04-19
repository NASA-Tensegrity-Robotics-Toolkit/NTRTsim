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
    nSquareClusters(6),  // 6 = number of squares on 12Bar. On SUPERball, the number of faces is 8.
    nHexaClusters(9), 
    musclesPerSquareCluster(4), // 4 = number of muscles per square. On SUPERball, the number is 3.
    musclesPerHexaCluster(6),
    groundRods(4)
{
    squareClusters.resize(nSquareClusters);
    for (int i=0; i<nSquareClusters; i++) {
        squareClusters[i].resize(musclesPerSquareCluster);
    }

    hexaClusters.resize(nHexaClusters);
    for (int i=0; i<nHexaClusters; i++) {
        hexaClusters[i].resize(musclesPerHexaCluster);
    }
}

/** Set the lengths of the muscles and initialize the learning adapter */
void T12Controller::onSetup(T12Model& subject)
{
    double dt = 0.0001;
    
    groundFace = 9999;

    //Set the initial length of every muscle in the subject
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    //std::cout << "Muscle.size: " << muscles.size() << std::endl;
    for (size_t i = 0; i < muscles.size(); ++i) {
	//std::cout << "Muscles: " << muscles[i] << std::endl;
        tgBasicActuator * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
        pMuscle->setControlInput(this->m_initialLengths, dt);
    }

    populateClusters(subject);
    initPosition = subject.getBallCOM();

    // DEBUGGING
    cout << "initPosition x: " << initPosition[1] << endl;
    cout << "initPosition y: " << initPosition[2] << endl;
    cout << "initPosition z: " << initPosition[3] << endl;
    cout << endl;

    //setupAdapter();
    initializeSineWaves(); // For muscle actuation

    vector<double> state(nSquareClusters); // For config file usage (including Monte Carlo simulations)
    vector< vector <double> > actions(nSquareClusters);

    /*TEST
    int faceNumber = NULL; 
    vector<double> cluster(4); 
    cluster[0] = 1, cluster[1] = 2, cluster[2] = 18, cluster[3] = 20;
    for( int i=0; i<4; i++ ) {
	cout << cluster[i] << endl;
    }
    //faceNumber = determineFace(&cluster);
    cout << "Face number: " << faceNumber << endl;*/

    //get the actions (between 0 and 1) from evolution (todo)
    //actions = evolutionAdapter.step(dt,state);
    
    //transform them to the size of the structure
    actions = transformActions(actions);

    cout << "actions.size(): " << actions.size() << endl;

    //apply these actions to the appropriate muscles according to the sensor values
    applyActions(subject, actions);
}

void T12Controller::onStep(T12Model& subject, double dt)
{
    double distanceMoved;

    //cout << "Entered onStep." << endl;
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    }
    m_totalTime+=dt;

    if( m_totalTime > m_startTime) {
        getGroundFace(subject); 
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
}

// So far, only score used for eventual fitness calculation of an Escape Model
// is the maximum distance from the origin reached during that subject's episode
void T12Controller::onTeardown(T12Model& subject) {
    std::cout << "Tearing down" << std::endl;

    std::vector<double> scores; //scores[0] == displacement, scores[1] == energySpent
    double distance = displacement(subject);
    //double energySpent = totalEnergySpent(subject);

    //Invariant: For now, scores must be of size 2 (as required by endEpisode())
    scores.push_back(distance);
    //scores.push_back(energySpent);

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
    vector< vector <double> > actions2D(nSquareClusters, vector<double>(4)); // Vector to be returned
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
                //cout << "action: " << actions2D[i][j] << endl;
            } else {
                actions[i][j] = actions[i][j]*(ranges[j])+mins[j];
                //cout << "action: " << actions[i][j] << endl;
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

    assert(actions.size() == squareClusters.size());

    // DEBUGGING
    cout << "Cluster size: " << squareClusters.size() << endl;

    // Apply actions by cluster
    for (size_t cluster = 0; cluster < squareClusters.size(); cluster++) {
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


void T12Controller::populateClusters(T12Model& subject) {

    int nMuscles = 36;
    int iMuscle = 0;
    vector <tgBasicActuator*> squareCol; // Vectors to create cluster matrices
  //	  squareClusters.push_back(squareCol);
    vector <tgBasicActuator*> hexaCol;  
//    hexaClusters.push_back(hexaCol);

    int i0 = 0, i1 = 0, i2 = 0, i3 = 0, i4 = 0, i5 = 0, i6 =0, i7 = 0, i8 = 0, i9 = 0, i10 = 0, i11 = 0, i12 = 0, i13 = 0;


    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles(); // Save all muscles in vector (could be done outside?)

    // Populate square clusters
    for(iMuscle = 0; iMuscle < nMuscles; iMuscle ++) {
        tgBasicActuator *const pMuscle = muscles[iMuscle];

        assert(pMuscle != NULL);
        cout << "iMuscle: " << iMuscle << ", muscle: " << pMuscle << endl;
        // Group muscles in clusters, square
        if (iMuscle == 1 || iMuscle == 2|| iMuscle == 18|| iMuscle == 20) { // Cluster 0
            squareClusters.at(0).push_back(pMuscle);
            squareClusters.at(i0).at(0) = pMuscle;
            i0++;
        } else if (iMuscle == 3 || iMuscle == 5 || iMuscle == 23 || iMuscle == 34) { // Cluster 1
            squareClusters.at(1).push_back(pMuscle);
            squareClusters.at(i1).at(1) = pMuscle;
            i1++;
        } else if (iMuscle == 7 || iMuscle == 8 || iMuscle == 25 || iMuscle == 30) { // Cluster 2
            squareClusters.at(2).push_back(pMuscle);
            squareClusters.at(i2).at(2) = pMuscle;
            i2++;
        } else if (iMuscle == 9 || iMuscle == 11 || iMuscle == 28 || iMuscle == 29) { // Cluster 3
            squareClusters.at(3).push_back(pMuscle);
            squareClusters.at(i3).at(3) = pMuscle;
            i3++;
        } else if (iMuscle == 12 || iMuscle == 13 || iMuscle == 21 || iMuscle == 31) { // Cluster 4
            squareClusters.at(4).push_back(pMuscle);
            squareClusters.at(i4).at(4) = pMuscle;
            i4++;
        } else if (iMuscle == 15 || iMuscle == 16 || iMuscle == 26 || iMuscle == 32) { // Cluster 5
            squareClusters.at(5).push_back(pMuscle);
            squareClusters.at(i5).at(5) = pMuscle;
            i5++;
        } else {
            cout << "Muscle is not related to the square faces." << endl;
        }
    }

    // DEBUGGING
    cout << "Square cluster: " << endl;
    for(int i=0; i<6; i++) {
	for(int j=0; j<4; j++){
		cout << squareClusters[j][i] << " ";
	}
        cout << endl;
    }
// @TODO
/* The following part needs to be re-defined, as the switch cases can't take input defined in another case
    // Populate hexagonial clusters
    for(iMuscle = 0; iMuscle < nMuscles; iMuscle ++) {
        tgBasicActuator *const pMuscle = muscles[iMuscle];

        assert(pMuscle != NULL);
        cout << "iMuscle: " << iMuscle << ", muscle: " << pMuscle << endl;
        // Group muscles in clusters, hexagon
        switch (iMuscle) {
             case (0 || 1 || 22 || 23 || 30 || 35): // Cluster 6
                 hexaClusters.at(0).push_back(pMuscle);
                 hexaClusters.at(i6).at(0) = pMuscle;
                 i6++;
             case ( 0 || 2 || 6 || 8 || 12 || 14): // Cluster 7
                 hexaClusters.at(1).push_back(pMuscle);
                 hexaClusters.at(i7).at(1) = pMuscle;
                 i7++;
             case (4 || 5 || 9 || 10 || 15 || 17): // Cluster 8
                 hexaClusters.at(2).push_back(pMuscle);
                 hexaClusters.at(i8).at(2) = pMuscle;
                 i8++;
             case (3 || 4 || 18 || 19 || 22 || 26): // Cluster 9
                 hexaClusters.at(3).push_back(pMuscle);
                 hexaClusters.at(i9).at(3) = pMuscle;
                 i9++;
             case (6 || 7 || 21 || 24 || 27 || 29):  // Cluster 10
                 hexaClusters.at(4).push_back(pMuscle);
                 hexaClusters.at(i10).at(4) = pMuscle;
                 i10++;
             case (13 || 14 || 19 || 20 || 32 || 33): // Cluster 11
                 hexaClusters.at(5).push_back(pMuscle);
                 hexaClusters.at(i11).at(5) = pMuscle;
                 i11++;
             case (16 || 17 || 27 || 28 || 31 || 33): // Cluster 12
                 hexaClusters.at(5).push_back(pMuscle);
                 hexaClusters.at(i12).at(6) = pMuscle;
                 i12++;
             case (10 || 11 || 24 || 25 || 34 || 35): // Cluster 13
                 hexaClusters.at(5).push_back(pMuscle);
                 hexaClusters.at(i13).at(7) = pMuscle;
                 i13++;
           }
        }
    }

    // DEBUGGGING
    cout << "Hexagon cluster: " << endl;
    for(int i=0; i<8; i++) {
	for(int j=0; j<6; j++){
		cout << hexaClusters[j][i] << " ";
	}
        cout << endl;
    }*/



}

/* Initializes sine waves, each cluster has identical parameters */
void T12Controller::initializeSineWaves() {
    amplitude = new double[nSquareClusters];
    angularFrequency = new double[nSquareClusters];
    phaseChange = new double[nSquareClusters]; // Does not use last value stored in array
    dcOffset = new double[nSquareClusters];    

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
        //std::cout<<"Cell " << i << ": " << result[i] << "\n";
        double seed = ((double) (rand() % 100)) / 100;
        result[i] += (0.01 * seed) - 0.005; // Value +/- 0.005 of original

        if(result[i] >= 1) {
            result[i] = result[i] - 0.5; // Dummy solution before values from learning is found for 12 bar
        }
      
        //std::cout<<"Cell " << i << ": " << result[i] << "\n";
    }

    cout << "readManualParams() finished. result.size = " << result.size() << endl;

    return result;
}

void T12Controller::printSineParams() {
    for (size_t cluster = 0; cluster < squareClusters.size(); cluster++) {
        cout << "amplitude[" << cluster << "]: " << amplitude[cluster] << endl;
        cout << "angularFrequency[" << cluster << "]: " << angularFrequency[cluster] << endl;
        cout << "phaseChange[" << cluster << "]: " << phaseChange[cluster] << endl;
        cout << "dcOffset[" << cluster << "]: " << dcOffset[cluster] << endl;
        cout << endl; 
    }     
}

// @ TODO: Implement for hexagon faces too
// For a certain rod configuration, determine corresponding face 
void T12Controller::determineFace() {

    // Cluster with respect to the rod number 
    vector<double> cluster0(4), cluster1(4), cluster2(4), cluster3(4), cluster4(4), cluster5(4);
    cluster0[0] = 0, cluster0[1] = 3, cluster0[2] = 7, cluster0[3] = 11;
    cluster1[0] = 0, cluster1[1] = 4, cluster1[2] = 8, cluster1[3] = 9;
    cluster2[0] = 1, cluster2[1] = 4, cluster2[2] = 6, cluster2[3] = 11;
    cluster3[0] = 1, cluster3[1] = 5, cluster3[2] = 8, cluster3[3] = 10;
    cluster4[0] = 2, cluster4[1] = 3, cluster4[2] = 6, cluster4[3] = 10;
    cluster5[0] = 2, cluster5[1] = 5, cluster5[2] = 7, cluster5[3] = 9;
  /*  cout << "groundRods[0] = " << groundRods[0] << endl;
    cout << "groundRods[1] = " << groundRods[1] << endl;
    cout << "groundRods[2] = " << groundRods[2] << endl;
    cout << "groundRods[3] = " << groundRods[3] << endl;

    cout << "Size of groundRods = " << groundRods.size() << endl;
*/ 
    assert(groundRods.size() == cluster0.size());

//    cout << "Facenumber before: " << faceNumber << endl;

    // Determine configuration 
    if(groundRods == cluster0) {
	groundFace = 0;
    } else if( groundRods == cluster1) {
	groundFace = 1;
    } else if( groundRods == cluster2) {
	groundFace = 2;
    } else if( groundRods == cluster3) {
	groundFace = 3;
    } else if( groundRods == cluster4) {
	groundFace = 4;
    } else if( groundRods == cluster5) {
	groundFace = 5;
    }

}

/* Function to determine which face the robot stands on */
void T12Controller::getGroundFace(T12Model& subject) {

        int i = 0;
        vector<double> nodePos(6);
	int oldGroundFace = groundFace;

        for( int rodNumber = 0; rodNumber<12; rodNumber++) {   // Get position of every endcap of rod
            nodePos = subject.getNodePosition(rodNumber);
	    //cout << "Checking rod number: " << rodNumber << endl;
		//cout << "size of nodePos: " << nodePos.size() << endl;
		//for(int j=0; j<nodePos.size(); j++){
		//    cout << "Position element [" << nodePos[j] << "]" << endl;
		//}
            if(nodePos[1] < 2 || nodePos[4] < 2) { // Saves rod number if endcap is close to ground
                groundRods[i] = rodNumber; // groundRods.push_back(rodNumber);
		//cout << "Rodnumber on ground: " << rodNumber << endl;
                i++;
            }
 	}

            if(groundRods.size() == 4) {
		//cout << "Robot is on a square face." << endl;
		determineFace();
	    } else if(groundRods.size() == 6) {
 	      //cout << "Robot is on hexagonal face." << endl;
 	      //faceNumber = determineFace(groundRods);
	    } else {
		groundFace = 9999;
		//cout << "Robot is on no particular face." << endl;
   	    }

    if(groundFace != oldGroundFace) {
	cout << "New face number, current ground face is: " << groundFace << endl;
        groundFaceHistory.push_back(groundFace);
        cout << "Absolute distance moved: " << displacement(subject) << endl;   
    } 
}



