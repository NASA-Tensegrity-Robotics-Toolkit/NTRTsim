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
//#include "learning/AnnealEvolution/AnnealEvolution.h"
//#include "learning/Adapters/AnnealAdapter.h"
//#include "learning/Configuration/configuration.h"
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
#include <sstream>
#include <ctime>

# define M_PI 3.14159265358979323846 
                               
using namespace std;

/* S E T T I N G S */
bool saveData = true; // Save parameters and result to file
bool useLearning = false; // True: Use learning (Monte Carlo), False: use parameters from file
bool tweakParams = false; // When reading parameters from file, tweak with up to 0.5% to optimize output from previous runs

//Constructor using the model subject and a single pref length for all muscles.
//Currently calibrated to decimeters
T12Controller::T12Controller(T12Model* subject, const double initialLength, double startTime) :
    m_initialLengths(initialLength),
    m_startTime(startTime),
    m_totalTime(0.0),
    maxStringLengthFactor(0.50),
    nSquareClusters(6),  // 6 = number of squares on 12Bar. On SUPERball, the number of faces is 8.
    nHexaClusters(9),  // 9 = number of hexagons on a 12Bar
    musclesPerSquareCluster(4), // 4 = number of muscles per square. On SUPERball, the number is 3.
    musclesPerHexaCluster(6) // 6 = number of muscles per hexagon
{

    // Create matrices, will be populated with muscles
    squareClusters.resize(nSquareClusters);
    for (int i=0; i<nSquareClusters; i++) {
        squareClusters[i].resize(musclesPerSquareCluster);
    }
    hexaClusters.resize(nHexaClusters);
    for (int i=0; i<nHexaClusters; i++) {
        hexaClusters[i].resize(musclesPerHexaCluster);
    }
}

/* Set the initial length of the muscles and initialize the learning adapter */
void T12Controller::onSetup(T12Model& subject)
{
    double dt = 0.0001;
    
    cout << "Current time is: " << m_totalTime << endl; // To verify each simulation starts with t = 0

    //Set the initial length of every muscle in the subject
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
        tgBasicActuator * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
        pMuscle->setControlInput(this->m_initialLengths, dt);
    }
   
    populateClusters(subject); // Populate the square clusters with muscles

    initPosition = subject.getBallCOM(); // Get initial position for calculation of distance travelledee
    // DEBUGGING
    /*cout << "initPosition x: " << initPosition[1] << endl;
    cout << "initPosition y: " << initPosition[2] << endl;
    cout << "initPosition z: " << initPosition[3] << endl;
    cout << endl;*/

 
    actions.resize(musclesPerSquareCluster, vector<double> (nSquareClusters, 0));
    for (int j = 0; j<nSquareClusters; j++) { 
	for (int i = 0; i<musclesPerSquareCluster; i++) {
 	    cout << actions[i][j] << " ";
        }
   	cout << endl;
    } 
    cout << endl;
    // If learning is used, setup adapter and learning parameters
    if(useLearning) randomizeParams();

    initializeSineWaves(); // For muscle actuation

    actions = transformActions(); // Transform the actions to right format

    //apply these actions to the appropriate muscles according to the sensor values
    applyActions(subject);

    printSineParams();
}

void T12Controller::onStep(T12Model& subject, double dt)
{
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    }
    m_totalTime+=dt;


    if( m_totalTime > m_startTime) {
        getGroundFace(subject); // check which face the robot is currently standing on 
        setPreferredMuscleLengths(subject, dt); 

        const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
        //Move motors for all the muscles
        for (size_t i = 0; i < muscles.size(); ++i)
        {
            tgBasicActuator * const pMuscle = muscles[i];
            assert(pMuscle != NULL);
            pMuscle->moveMotors(dt);
        }
//        cout << m_totalTime << endl;
	//DEBUGGING 
	if (m_totalTime > 60 && m_totalTime < 60.1) cout << "Distance after 60 s: " << displacement(subject) << endl; 
    }
}

// Things to be done in between each simulation, calculate scores, save data and clear parameters
void T12Controller::onTeardown(T12Model& subject) {
    cout << endl << "Tearing down controller" << endl;

    energySpent = totalEnergySpent(subject);
    cout << "Energy spent: " << energySpent << endl;
    
    if(saveData) {
    	saveData2File();
	cout << "Data saved." << endl;
    }

    clearParams(); // Clear all parameters for next simulation
    
    // update simulation number
    simulationNumber++;
    cout << "Simulation number: " << simulationNumber << endl << endl;

    // If any of subject's dynamic objects need to be freed, this is the place to do so
}

/** 
 * Returns the modified actions vector such that 
 * each action value is now scaled to fit the model
 * Invariant: actions[x].size() == 4 for all legal values of x
 * Invariant: Each actions[] contains: amplitude, angularFrequency, phase, dcOffset
 */
vector< vector <double> > T12Controller::transformActions()
{ 
    // DEBUGGING 
    for(int j=0;j<nSquareClusters;j++) { //6x, number of rows
        for (int i=0; i<musclesPerSquareCluster; i++) { //4x, number of columns
            cout << actions[i][j] << " ";
        }
        cout << endl;
    }
    cout << "\n";


    vector< vector <double> > adaptedActions(musclesPerSquareCluster, vector<double>(nSquareClusters, 0)); // Vector to be returned
    adaptedActions = actions;

    assert(adaptedActions.size() == actions.size());

    // If reading parameters from file, do this
    if(!useLearning) { 
       vector <double> manualParams(24, 1); // '4' for the number of sine wave parameters, nClusters = 6 -> 24 total
        const char* filename = "/home/hannah/Projects/NTRTsim/src/dev/hpetersson/12BarTensegrity/InputActions/actions_b_5.csv";
        std::cout << "Using manually set parameters from file " << filename << endl; 
        int lineNumber = 1;
        manualParams = readManualParams(lineNumber, filename);  
/*	for(int i = 0; i<squareClusters.size(); i++) {  // Assign sine parameters 
	    amplitude[i] = manualParams[i];
	    angularFrequency[i] = manualParams[i+squareClusters.size()];
	    phase[i] = manualParams[i+2*squareClusters.size()];
	    dcOffset[i] = manualParams[i+3*squareClusters.size()];
	}*/

	int k = 0; // Assign actions (same as sine parameters, done for completeness)
	for(int j = 0; j<musclesPerSquareCluster; j++) {
	    for(int i = 0; i<nSquareClusters; i++) {
	    adaptedActions[j][i] = manualParams[k];
	    cout << adaptedActions[j][i] << " ";
	    k++; 
    	    }
	    cout << endl; 
        }
	cout << endl; 
    }

    // If learning is used, do this
    if(useLearning) {
//    double pretension = 0.9; // Tweak this value if need be. What is this actually?

         // Minimum amplitude, angularFrequency, phase, and dcOffset
        double mins[4]  = {0.3, 
                           0, // dummy 
                           -1 * M_PI, 
                           m_initialLengths};// * (1 - maxStringLengthFactor)};

        // Maximum amplitude, angularFrequency, phase, and dcOffset
        double maxes[4] = {m_initialLengths/2, 
                           0, // dummy
                           M_PI, 
                           m_initialLengths};// * (1 + maxStringLengthFactor)}; 

        assert((maxes[0]-mins[0])>0);
        double ranges[4] = {maxes[0]-mins[0], maxes[1]-mins[1], maxes[2]-mins[2], maxes[3]-mins[3]};

        for(int j=0;j<nSquareClusters;j++) { //6x, number of rows
            for (int i=0; i<musclesPerSquareCluster; i++) { //4x, number of columns
                cout << actions[i][j] << " ";
            }
            cout << endl;
        }
  	cout << "\n";

        // Apply output of learing to all parameters but the angular frequency (since the maximum angular frequency is dependent on the amplitude)
        for(int j=0;j<nSquareClusters;j++) { //6x, number of rows
            for (int i=0; i<musclesPerSquareCluster; i++) { //4x, number of columns
                adaptedActions[i][j] = actions[i][j]*(ranges[i])+mins[i];
                cout << adaptedActions[i][j] << " ";
            }
	    cout << endl;
        }
        cout << endl; 

        // Find maximum angular frequency with the help of the amplitude 
        double maxMotorVelocity = 1; // Reasonable values would be 5-10 cm/s --> 0.5/1 dm/s (SUPERball has 2 cm/s)
        double maxAngFrequencies[6];
        double minAngFrequencies[6] = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3}; // Appropriate
        double rangeAngFrequencies[6];	
        vector<double> amps(6);
        for(int i = 0; i <nSquareClusters; i++) { 
	    amps[i] = adaptedActions[0][i];
	    cout << "amps: " << amps[i] << endl;
            maxAngFrequencies[i] = M_PI * maxMotorVelocity / amps[i]; // Frequency limit is based on motor velocity
            rangeAngFrequencies[i] = maxAngFrequencies[i] - minAngFrequencies[i];
        }
        
        // Apply output of learning to the angular frequency parameters (since the maximum angular frequency is dependent on the amplitude)
        for(int i=0;i<nSquareClusters;i++) { //6x
            adaptedActions[1][i] = actions[1][i]*(rangeAngFrequencies[i])+minAngFrequencies[i];
        }
    }
      for(int j=0;j<nSquareClusters;j++) { //6x, number of rows
            for (int i=0; i<musclesPerSquareCluster; i++) { //4x, number of columns
                cout << adaptedActions[i][j] << " ";
            }
	    cout << endl;
        }
        cout << endl; 
    return adaptedActions;
}

void T12Controller::randomizeParams() { 
    
    if (simulationNumber == 0) srand(time(NULL));

    int nParams = nSquareClusters * musclesPerSquareCluster;
    cout << "Number of inputs: " << nParams << endl;

    for(int j=0;j<nSquareClusters;j++) { //6x
        for (int i=0; i<musclesPerSquareCluster; i++) { //4x
            actions[i][j] = ((double) rand() / RAND_MAX);
            cout << actions[i][j] << " ";
        }
	cout << endl;
    }
    cout << "Random parameters obtained." << endl;
}



/**
 * Defines each cluster's sine wave according to actions
 */
void T12Controller::applyActions(T12Model& subject)
{

    assert(actions.size() == squareClusters[0].size());

    // Apply actions by cluster
    for (size_t cluster = 0; cluster < squareClusters.size(); cluster++) {
        amplitude[cluster] = actions[0][cluster]; 
  	angularFrequency[cluster] = actions[1][cluster];
        phase[cluster] = actions[2][cluster];
        dcOffset[cluster] = actions[3][cluster];
    }
}


/*void T12Controller::setupAdapter() {
    string suffix = "_12Bar";
    string configAnnealEvolution = "Config.ini";
    AnnealEvolution* evo = new AnnealEvolution(suffix, configAnnealEvolution, "", simulationNumber);
    bool isLearning = true;
    configuration configEvolutionAdapter;
    configEvolutionAdapter.readFile(configAnnealEvolution);

    evolutionAdapter.initialize(evo, isLearning, configEvolutionAdapter);

}*/



void T12Controller::setPreferredMuscleLengths(T12Model& subject, double dt) {

    // Calculate the new length, one cluster at a time, and apply it to that cluster
    for(int j = 0; j < squareClusters.size(); j++) { // squareClusters.size() gives number of columns
	double newLength = amplitude[j] * sin(angularFrequency[j] * m_totalTime + phase[j]) + dcOffset[j];
        double minLength = m_initialLengths * (1-maxStringLengthFactor);
        double maxLength = m_initialLengths * (1+maxStringLengthFactor);
        
        if (newLength <= minLength) {
	    //cout << "Hit minLength limit." << endl;
            newLength = minLength;
        } else if (newLength >= maxLength) {
	    //cout << "Hit maxLength limit." << endl;
            newLength = maxLength;
        }

	for(int i = 0; i < squareClusters[0].size(); i++) { // squareClusters[0].size gives number of rows
	    assert(squareClusters[j][i] != NULL);
	    tgBasicActuator *const pMuscle = squareClusters[j][i];
 	    assert(pMuscle != NULL);

            pMuscle->setControlInput(newLength, dt);
	    
        } 
     
	// DEBUGGING
        /*cout << "Square cluster: " << endl;
        for(int j=0; j<6; j++) {
	    for(int i=0; i<4; i++){
		cout << squareClusters[i][j] << " ";
	    }    
            cout << endl;
        }*/
   }
}


/* Populate matrices with mucsles. */
void T12Controller::populateClusters(T12Model& subject) {
    int nMuscles = 36;
    int iMuscle = 0;
  
    vector <tgBasicActuator*> squareCol; // Vectors to create cluster matrices
    //vector <tgBasicActuator*> hexaCol; // Hexagons will not be actuated in this application 

    int i0 = 0, i1 = 0, i2 = 0, i3 = 0, i4 = 0, i5 = 0;


    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles(); // Save all muscles in vector

    // Populate square clusters
    for(iMuscle = 0; iMuscle < nMuscles; iMuscle ++) {
        tgBasicActuator *const pMuscle = muscles[iMuscle];

        assert(pMuscle != NULL);
        
	// Group muscles in clusters, square
        if (iMuscle == 1 || iMuscle == 2|| iMuscle == 18|| iMuscle == 20) { // Cluster 0
            squareClusters[0][i0] = pMuscle;
            i0++;
        } else if (iMuscle == 3 || iMuscle == 5 || iMuscle == 23 || iMuscle == 34) { // Cluster 1
            squareClusters[1][i1] = pMuscle;
            i1++;
        } else if (iMuscle == 7 || iMuscle == 8 || iMuscle == 25 || iMuscle == 30) { // Cluster 2
            squareClusters[2][i2] = pMuscle;
            i2++;
        } else if (iMuscle == 9 || iMuscle == 11 || iMuscle == 28 || iMuscle == 29) { // Cluster 3
            squareClusters[3][i3] = pMuscle;
            i3++;
        } else if (iMuscle == 12 || iMuscle == 13 || iMuscle == 21 || iMuscle == 31) { // Cluster 4
            squareClusters[4][i4] = pMuscle;
            i4++;
        } else if (iMuscle == 15 || iMuscle == 16 || iMuscle == 26 || iMuscle == 32) { // Cluster 5
            squareClusters[5][i5] = pMuscle;
            i5++;
        }
    }
}

/* Initializes sine waves, each cluster has identical parameters */
void T12Controller::initializeSineWaves() {
    amplitude = new double[nSquareClusters];
    angularFrequency = new double[nSquareClusters];
    phase = new double[nSquareClusters];
    dcOffset = new double[nSquareClusters];    
}

/* Calculates the distance travelled with reference to initial position */
double T12Controller::displacement(T12Model& subject) {
    vector<double> finalPosition = subject.getBallCOM();

    //assert(finalPosition[0] > 0); //Negative y-value indicates a flaw in the simulator that run (tensegrity went 'underground')

    const double newX = finalPosition[0];
    const double newZ = finalPosition[2];
    const double oldX = initPosition[0];
    const double oldZ = initPosition[2];

    const double distanceMoved = sqrt((newX-oldX) * (newX-oldX) + 
                                      (newZ-oldZ) * (newZ-oldZ));
    return distanceMoved;
}

/* Function to read parameters from file, and if desired, tweak input parameters with up to 0.5% */
std::vector<double> T12Controller::readManualParams(int lineNumber, const char* filename) {
    assert(lineNumber > 0);
    vector<double> result(24, 1.0);
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

    // Split line into parameters
    stringstream lineStream(line);
    string cell;
    int iCell = 0;
    while(getline(lineStream,cell,',')) {
        result[iCell] = atof(cell.c_str());
        iCell++;
    }

    if (simulationNumber > 10) { // Don't tweak for the first 10 simulation to ensure consistency
        // Tweak each read-in parameter by as much as 0.5% (params range: [0,1])     
        if (tweakParams) {
            cout << "Tweaking parameters from file with up to 0.5%." << endl;
            for (int i=0; i < result.size() - 6; i++) {
                std::cout<<"Cell " << i << ": " << result[i];
                double seed = ((double) (rand() % 100)) / 100;
                result[i] += (0.01 * seed) - 0.005; // Value +/- 0.005 of original
	        cout << ", tweaked: " << result[i] << endl;
            }
        } else cout << "Actions not tweaked for the first 10 simulations." << endl;
    }
    return result;
}

void T12Controller::printSineParams() {
    cout << endl;
    for (size_t cluster = 0; cluster < squareClusters.size(); cluster++) {
        cout << "amplitude[" << cluster << "]: " << amplitude[cluster] << endl;
        cout << "angularFrequency[" << cluster << "]: " << angularFrequency[cluster] << endl;
        cout << "phase[" << cluster << "]: " << phase[cluster] << endl;
        cout << "dcOffset[" << cluster << "]: " << dcOffset[cluster] << endl;
        cout << endl; 
    }     
}

// For a certain rod configuration, determine corresponding face 
void T12Controller::determineFace(bool isSquareFace) {

    // Cluster with respect to the rod number 
    vector<double> cluster0(4), cluster1(4), cluster2(4), cluster3(4), cluster4(4), cluster5(4);
    cluster0[0] = 0, cluster0[1] = 3, cluster0[2] = 7, cluster0[3] = 11;
    cluster1[0] = 0, cluster1[1] = 4, cluster1[2] = 8, cluster1[3] = 9;
    cluster2[0] = 1, cluster2[1] = 4, cluster2[2] = 6, cluster2[3] = 11;
    cluster3[0] = 1, cluster3[1] = 5, cluster3[2] = 8, cluster3[3] = 10;
    cluster4[0] = 2, cluster4[1] = 3, cluster4[2] = 6, cluster4[3] = 10;
    cluster5[0] = 2, cluster5[1] = 5, cluster5[2] = 7, cluster5[3] = 9;

    vector<double> cluster6(6), cluster7(6), cluster8(6), cluster9(6), cluster10(6), cluster11(6), cluster12(6), cluster13(6);
    cluster6[0] = 0, cluster6[1] = 4, cluster6[2] = 6, cluster6[3] = 7, cluster6[4] = 8, cluster6[5] = 11;    
    cluster7[0] = 0, cluster7[1] = 1, cluster7[2] = 2, cluster7[3] = 3, cluster7[4] = 6, cluster7[5] = 11;
    cluster8[0] = 0, cluster8[1] = 1, cluster8[2] = 2, cluster8[3] = 5, cluster8[4] = 8, cluster8[5] = 9;
    cluster9[0] = 0, cluster9[1] = 3, cluster9[2] = 4, cluster9[3] = 5, cluster9[4] = 7, cluster9[5] = 9;
    cluster10[0] = 1, cluster10[1] = 3, cluster10[2] = 4, cluster10[3] = 5, cluster10[4] = 6, cluster10[5] = 10;
    cluster11[0] = 2, cluster11[1] = 3, cluster11[2] = 7, cluster11[3] = 9, cluster11[4] = 10, cluster11[5] = 11;
    cluster12[0] = 2, cluster12[1] = 5, cluster12[2] = 6, cluster12[3] = 7, cluster12[4] = 8, cluster12[5] = 10;
    cluster13[0] = 1, cluster13[1] = 4, cluster13[2] = 8, cluster13[3] = 9, cluster13[4] = 10, cluster13[5] = 11;

   if(isSquareFace) {
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
        } else {
	  //  groundFace = -1;
    	}
    } else {
	if(groundRods == cluster6) {
	    groundFace = 6;
 	} else if( groundRods == cluster7) {
	    groundFace = 7;
	} else if( groundRods == cluster8) {
	    groundFace = 8;
	} else if( groundRods == cluster9) { 
	    groundFace = 9;
	} else if( groundRods == cluster10) { 
	    groundFace = 10;
	} else if( groundRods == cluster11) {
	    groundFace = 11;
	} else if( groundRods == cluster12) { 
	    groundFace = 12;
 	} else if( groundRods == cluster13) { 
	    groundFace = 13;
	} else {
	    //groundFace = -1;
	}
    }

}

// Calculate energy spent
double T12Controller::totalEnergySpent(T12Model& subject) {

    double totalEnergySpent=0;

    vector<tgBasicActuator* > tmpStrings = subject.getAllMuscles();
    assert(tmpStrings[0] != NULL);

    for(int i=0; i<tmpStrings.size(); i++)
    {
	tgSpringCableActuator::SpringCableActuatorHistory stringHist = tmpStrings[i]->getHistory();

        for(int j=1; j<stringHist.tensionHistory.size(); j++)
        {
            const double previousTension = stringHist.tensionHistory[j-1];
            const double previousLength = stringHist.restLengths[j-1];
            const double currentLength = stringHist.restLengths[j];

	    // DEBUGGING	
// 	    cout << "Previous tension: " << previousTension << endl;
// 	    cout << "Previous length: " << previousLength << endl;
// 	    cout << "Current length: " << currentLength << endl;

            //TODO: examine this assumption - free spinning motor may require more power         
            double motorSpeed = (currentLength-previousLength);
            if(motorSpeed > 0) // Assumption: No energy is required to elongate cable
                motorSpeed = 0;
            const double workDone = - previousTension * motorSpeed;
            totalEnergySpent += workDone;
        }
    }
    return totalEnergySpent;
}



/* Function to determine which face the robot stands on */
void T12Controller::getGroundFace(T12Model& subject) {

        int i = 0;
        vector<double> nodePos(6);
	double oldGroundFace = groundFace;

        for( int rodNumber = 0; rodNumber<12; rodNumber++) {   // Get position of every endcap of rod
            nodePos = subject.getNodePosition(rodNumber);
            if(nodePos[1] < 2 || nodePos[4] < 2) { // Saves rod number if endcap is close to ground
                groundRods.push_back(rodNumber);//groundRods[i] = rodNumber; //groundRods.push_back(rodNumber);
            }
 	}

            if(groundRods.size() == 4) {
	        // cout << "Robot is on a square face." << endl;
		determineFace(true);
	    } else if(groundRods.size() == 6) {
 	      //cout << "Robot is on hexagonal face." << endl;
 	      determineFace(false);
	    } else {
		//cout << "Robot is on no particular face." << endl;
   	    }

    groundRods.clear();

    double oldManhattan = distanceMovedManhattan;
    distanceMovedManhattan = displacement(subject);
    distanceMovedTotal += abs(oldManhattan - distanceMovedManhattan); 
    if(groundFace != oldGroundFace) { // && groundFace != -1) {
        groundFaceHistory.push_back(groundFace); // Save ground face in history log
    } 
}

/* Function for writing data to txt file */
void T12Controller::write2txtFile(double contentDouble, char const* contentString, bool isDouble) {
    
    ofstream myFile(txtPath.c_str(), ios::app);
    if(isDouble) {
        myFile << contentDouble;
    } else {
        myFile << contentString;
    }
    myFile.close();

    //cout << "Writing to file completed." << endl;
}

/* Function for writing data to csv file 
contentDouble - double to be written
sign - either comma or \n
isDouble - indicating which type to write
*/
void T12Controller::write2csvFile(double contentDouble, char const* sign, bool isDouble) {
    
    ofstream myFile(csvPath.c_str(), ios::app);
    if(isDouble) {
        myFile << contentDouble;
    } else {
        myFile << sign;
    }
    myFile.close();

//    cout << "Writing to csv file completed." << endl;
}


/* Get the .csv and .txt file name for saved data, name based on current date and time */
void T12Controller::getFileName(void) {

    time_t t = time(0);
    tm* now = localtime(&t);
 
    string txttemp = ".txt";
    string csvtemp = ".csv";

    ostringstream txt_path_out(txttemp);
    ostringstream csv_path_out(csvtemp);

    txt_path_out << "/home/hannah/Projects/NTRTsim/src/dev/hpetersson/12BarTensegrity/outputFiles/textgen_b3_";
    csv_path_out << "/home/hannah/Projects/NTRTsim/src/dev/hpetersson/12BarTensegrity/outputFiles/gen_b3_";

    
    time_t year = (now->tm_year + 1900);
    time_t month = (now->tm_mon + 1);
    time_t day = (now->tm_mday);
    time_t hour = (now->tm_hour);
    time_t min = (now->tm_min);
    time_t sec = (now->tm_sec);

    txt_path_out << year << "0" << month << "0" << day << "_" << hour << "h" << min << "m" << sec << "s" << ".txt";
    csv_path_out << year << "0" << month << "0" << day << "_" << hour << "h" << min << "m" << sec << "s" << ".csv";
 
    txtPath = txt_path_out.str();
    csvPath = csv_path_out.str();

    // Print header for csv file
    write2csvFile(0, "Simulation number,Manhattan distance,Snirky distance,Energy spent,", 0);
    for (int i = 0; i < actions[0].size(); i++) {
	for (int j = 0; j < actions.size(); j++) { // First, print all elements in the row  
            write2csvFile(0, "actions[", 0);       // When done, switch to next row
	    write2csvFile(j, "", 1);
	    write2csvFile(0, "][", 0);
	    write2csvFile(i, "", 1);
  	    write2csvFile(0, "],", 0);
	}
    }	 
    for (int i = 0; i < nSquareClusters; i++) { // Print sine params
	write2csvFile(0, "amplitude[", 0);
	write2csvFile(i, "", 1);
	write2csvFile(0, "],angularFrequency[", 0);
	write2csvFile(i, "", 1);
	write2csvFile(0, "],phase[", 0);
	write2csvFile(i, "", 1);
	write2csvFile(0, "],dcOffset[", 0);
	write2csvFile(i, "", 1);
	write2csvFile(0, "],", 0);
    }

    write2csvFile(0, "Initial Length,Start time,Faces", 0);
    write2csvFile(0, "\n", 0);
}



void T12Controller::saveData2File() { 

    // DEBUGGING
    /*cout << "initPosition x: " << initPosition[1] << endl;
    cout << "initPosition y: " << initPosition[2] << endl;
    cout << "initPosition z: " << initPosition[3] << endl;
    cout << endl;*/

// WRITE TO TEXT FILE FOR EASY ACCESS TO DATA
    // Sine params
    write2txtFile(0, "Index - Amplitude - Angular Frequency - Phase Change - DC Offset", 0); 
    write2txtFile(0, "\n", 0);
    for (int i = 0; i < nSquareClusters; i++) {
	write2txtFile(i, "", 1);
	write2txtFile(0, "         ", 0);
	write2txtFile(amplitude[i], "", 1);
	write2txtFile(0, "         ", 0);
	write2txtFile(angularFrequency[i], "", 1);
	write2txtFile(0, "         ", 0);
	write2txtFile(phase[i], "", 1);
	write2txtFile(0, "         ", 0);
	write2txtFile(dcOffset[i], "", 1);
	write2txtFile(0, "\n", 0);
    }

    write2txtFile(0, "\n", 0);
    
    // Actions
    write2txtFile(0, "Actions", 0); 
    write2txtFile(0, "\n", 0);
    for (int i = 0; i < actions[0].size(); i++) {
	for (int j = 0; j < actions.size(); j++) {  
            write2txtFile(actions[j][i], "", 1);
	    write2txtFile(0, ", ", 0);
	}
	write2txtFile(0, "\n", 0);
    }

    write2txtFile(0, "\n", 0);
    
    // ball COM distance
    write2txtFile(0, "Total displacement: ", 0);
    write2txtFile(distanceMovedManhattan, "", 1);

    write2txtFile(0, "\n", 0);
    write2txtFile(0, "\n", 0);

    // faces (including hexa)
    write2txtFile(0, "Ground face sequence: ", 0);

    for (int i = 0; i < groundFaceHistory.size(); i++) {
	write2txtFile(groundFaceHistory[i], "", 1);
	write2txtFile(0, ", ", 0);
    }

    write2txtFile(0, "\n", 0);
    write2txtFile(0, "\n", 0);
    
    // Total energy spent
    write2txtFile(0, "Total energy spent: ", 0);
    write2txtFile(energySpent, "", 1);
    

// WRITE TO CSV FILE FOR MATLAB EVALUATION

    // simulation number
    write2csvFile(simulationNumber,"",1);
    write2csvFile(0,",",0);

    // manhattan distance moved
    write2csvFile(distanceMovedManhattan,"",1);
    write2csvFile(0,",",0);
   
    // snirky distance moved
    cout << "Manhattan distance moved: " << distanceMovedManhattan << endl; 
    cout << "Total distance moved: " << distanceMovedTotal << endl; 
    write2csvFile(distanceMovedTotal,"",1);
    write2csvFile(0,",",0);

    // energy spent 
    write2csvFile(energySpent,"",1);
    write2csvFile(0,",",0);

    // actions
    for (int i = 0; i < actions[0].size(); i++) {
	for (int j = 0; j < actions.size(); j++) {  
            write2csvFile(actions[j][i], "", 1);
	    write2csvFile(0, ",",  0);
	}
    }	

    // sine params
    for (int i = 0; i < nSquareClusters; i++) {
	write2csvFile(amplitude[i], "", 1);
	write2csvFile(0, ",", 0);
	write2csvFile(angularFrequency[i], "", 1);
	write2csvFile(0, ",", 0);
	write2csvFile(phase[i], "", 1);
	write2csvFile(0, ",", 0);
	write2csvFile(dcOffset[i], "", 1);
	write2csvFile(0, ",", 0);
    }

    // Initial length
    write2csvFile(m_initialLengths, "", 1);
    write2csvFile(0, ",", 0);
    

    // Start time
    write2csvFile(m_startTime, "", 1);
    write2csvFile(0, ",", 0);


    // ground face
    for(int i = 1; i < groundFaceHistory.size(); i++) {
        write2csvFile(groundFaceHistory[i], "", 1);
 	write2csvFile(0, ",", 0);
    }
    write2csvFile(0, "\n", 0);

}

void T12Controller::clearParams(void) { 
   
    cout << "Current time is: " << m_totalTime << endl;
    groundFaceHistory.clear();
    distanceMovedManhattan = 0;
    distanceMovedTotal = 0;
    energySpent = 0;
    m_totalTime = 0; 
    groundFace = 0;
    initPosition.clear();

    for (int i = 0; i < nSquareClusters; i++) {
	amplitude[i] = 0;
	angularFrequency[i] = 0;
	phase[i] = 0;
	dcOffset[i] = 0;
    }

    actions.clear();

}/*
	    amplitude[0] = 0.346153;
	    angularFrequency[0] = 5.68188;
	    phase[0] = 0.852517;
	    dcOffset[0] = 1;

	    amplitude[1] = 0.444575;
	    angularFrequency[1] = 4.03549;
	    phase[1] = 1.77779;
	    dcOffset[1] = 1;

	    amplitude[2] = 0.446121;
	    angularFrequency[2] = 5.99772;
	    phase[2] = 2.06179;
	    dcOffset[2] = 1;

	    amplitude[3] = 0.316079;
	    angularFrequency[3] = 9.36477;
	    phase[3] = 2.73579;
	    dcOffset[3] = 1;
	    
 	    amplitude[4] = 0.460203;
	    angularFrequency[4] = 0.853921;
	    phase[4] = 0.467104;
	    dcOffset[4] = 1;

	    amplitude[5] = 0.422964;
	    angularFrequency[5] = 4.51772;
	    phase[5] = -0.579297;
	    dcOffset[5] = 1;*/
    
