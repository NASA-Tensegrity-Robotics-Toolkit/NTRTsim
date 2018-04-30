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
#include <sstream>
#include <ctime>

# define M_PI 3.14159265358979323846 
                               
using namespace std;

/* S E T T I N G S */
bool saveData = true; // Save data to file
bool useLearning = true; // Use learning alt. use parameters from file

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
    groundRods(4) // Used for determining which face is on ground
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
    
    groundFace = -1;

    if(saveData) {
        getFileName();
        cout << "Will save to file " << path << endl;
        cout << endl;
        cout << "Why do you go here twice" << endl;
    }

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
    /*cout << "initPosition x: " << initPosition[1] << endl;
    cout << "initPosition y: " << initPosition[2] << endl;
    cout << "initPosition z: " << initPosition[3] << endl;
    cout << endl;*/

    if(useLearning) { 
        setupAdapter();
	cout << "Adapter finished setting up." << endl;
        vector<double> state(nSquareClusters); // For config file usage (including Monte Carlo simulations)
        //get the actions (between 0 and 1) from evolution
        actions = evolutionAdapter.step(dt,state);
        for(int k = 0; k < actions.size(); k++) {
	    for(int l = 0; l < actions[0].size(); l++) {
                cout << "actions[" << l << "][" << k << "]" << actions[l][k] << endl;
            }
	}
    } else {
    	vector< vector<double> > actions;
    }

    initializeSineWaves(); // For muscle actuation

    //transform them to the size of the structure
    actions = transformActions(actions);

    //apply these actions to the appropriate muscles according to the sensor values
    applyActions(subject, actions);
}

void T12Controller::onStep(T12Model& subject, double dt)
{
    double distanceMoved;

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
       /* for(int i=0; i<muscles.size(); i++)
        {
            vector<double> tmp;
            for(int j=0;j<2;j++)
            {
                tmp.push_back(0.5);
            }
            actions.push_back(tmp);
        }*/
    }
}

// So far, only score used for eventual fitness calculation of an Escape Model
// is the maximum distance from the origin reached during that subject's episode
void T12Controller::onTeardown(T12Model& subject) {
    std::cout << "Tearing down controller" << std::endl;

    //std::vector<double> scores; //scores[0] == displacement, scores[1] == energySpent
    //double distance = displacement(subject);
    //double energySpent = totalEnergySpent(subject);

    //Invariant: For now, scores must be of size 2 (as required by endEpisode())
    //scores.push_back(distance);
    //scores.push_back(energySpent);

    if(saveData) {
    	saveData2File();
	cout << "Data saved." << endl;
    }

    //evolutionAdapter.endEpisode(scores);

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
    vector <double> manualParams(24, 1); // '4' for the number of sine wave parameters, nClusters = 6 -> 24 total

    if(!useLearning) { 
        const char* filename = "logs/paramSortedBestTrials.dat";
        std::cout << "Using manually set parameters from file " << filename << endl; 
        int lineNumber = 1;
        manualParams = readManualParams(lineNumber, filename);  
        cout << "manualParams.size(): " << manualParams.size() << endl;
    }

    double pretension = 0.9; // Tweak this value if need be. What is this actually?

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

    // DEBUGGING
    // cout << "Actions matrix is of size: (" << actions2D.size() << ", " << actions2D[0].size() << ")" << endl;

    for(int i=0;i<actions2D.size();i++) { //6x
        for (int j=0; j<actions2D[i].size(); j++) { //4x
            if (!useLearning) {
                actions2D[i][j] = manualParams[i*actions2D[i].size() + j]*(ranges[j])+mins[j];
                //cout << "action: " << actions2D[i][j] << endl;
            } else if(useLearning) {
                actions2D[i][j] = actions1D[i][j]*(ranges[j])+mins[j];
                cout << "action2D: " << actions2D[i][j] << endl;
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
    //cout << "Action size: " << actions.size() << endl;
    //cout << "Cluster size: " << squareClusters.size() << endl;

    assert(actions.size() == squareClusters.size());

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
    string suffix = "_12Bar";
    string configAnnealEvolution = "Config.ini";
    AnnealEvolution* evo = new AnnealEvolution(suffix, configAnnealEvolution);
    bool isLearning = true;
    configuration configEvolutionAdapter;
    configEvolutionAdapter.readFile(configAnnealEvolution);

    evolutionAdapter.initialize(evo, isLearning, configEvolutionAdapter);
    cout << "We came this far" << endl;

}



void T12Controller::setPreferredMuscleLengths(T12Model& subject, double dt) {

    double phase = 0;

    for(int j = 0; j < squareClusters.size(); j++) { // squareClusters.size() gives number of rows

	double newLength = amplitude[j] * sin(angularFrequency[j] * m_totalTime + phase) + dcOffset[j];
        double minLength = m_initialLengths * (1-maxStringLengthFactor);
        double maxLength = m_initialLengths * (1+maxStringLengthFactor);
        
        if (newLength <= minLength) {
            newLength = minLength;
        } else if (newLength >= maxLength) {
            newLength = maxLength;
        }

	for(int i = 0; i < (squareClusters[0].size())/2; i++) { // squareClusters[0].size gives number of int columns (= # of double*2)
//	    cout << "[i ,j]: " << i << j << endl;
	    assert(squareClusters[i][j] != NULL);
	    tgBasicActuator *const pMuscle = squareClusters[i][j];
 	    assert(pMuscle != NULL);

            pMuscle->setControlInput(newLength, dt);
	    
        } 
        phase += phaseChange[j];

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

void T12Controller::populateClusters(T12Model& subject) {
    int nMuscles = 36;
    int iMuscle = 0;
  
    vector <tgBasicActuator*> squareCol; // Vectors to create cluster matrices
    // squareClusters.push_back(squareCol);
    vector <tgBasicActuator*> hexaCol;  
    // hexaClusters.push_back(hexaCol);

    int i0 = 0, i1 = 0, i2 = 0, i3 = 0, i4 = 0, i5 = 0;


    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles(); // Save all muscles in vector

    // Populate square clusters
    for(iMuscle = 0; iMuscle < nMuscles; iMuscle ++) {
        tgBasicActuator *const pMuscle = muscles[iMuscle];

        assert(pMuscle != NULL);
        
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
    //        cout << "Muscle is not related to the square faces." << endl;
        }
    }

    
    // DEBUGGING
    cout << "Square cluster: " << endl;
    for(int j=0; j<6; j++) {
	for(int i=0; i<4; i++){
		cout << squareClusters[i][j] << " ";
	}
        cout << endl;
    }
}

/* Initializes sine waves, each cluster has identical parameters */
void T12Controller::initializeSineWaves() {
    amplitude = new double[nSquareClusters];
    angularFrequency = new double[nSquareClusters];
    phaseChange = new double[nSquareClusters]; // Does not use last value stored in array
    dcOffset = new double[nSquareClusters];    

    // DEBUGGING
    cout << " amplitude: " << amplitude << " angularFrequency: " << angularFrequency << " phaseChange: " << phaseChange << " dcOffset: " << dcOffset << endl;
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

// @ TODO: Implement for hexagon faces
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

/* Function to determine which face the robot stands on */
void T12Controller::getGroundFace(T12Model& subject) {

        int i = 0;
        vector<double> nodePos(6);
	double oldGroundFace;

  	if(groundFace != -1) {
	    oldGroundFace = groundFace;
	}

        for( int rodNumber = 0; rodNumber<12; rodNumber++) {   // Get position of every endcap of rod
            nodePos = subject.getNodePosition(rodNumber);
            if(nodePos[1] < 2 || nodePos[4] < 2) { // Saves rod number if endcap is close to ground
                groundRods[7] = rodNumber; // groundRods.push_back(rodNumber);
		//cout << "Rodnumber on ground: " << rodNumber << endl;
                i++;
            }
 	}
        //cout << "groundRods.size = " << groundRods.size() << endl;
            if(groundRods.size() == 4) {
		//cout << "Robot is on a square face." << endl;
		determineFace(1);
	    } else if(groundRods.size() == 6) {
 	      cout << "Robot is on hexagonal face." << endl;
 	      determineFace(0);
	    } else {
	        // groundFace = -1;
		//cout << "Robot is on no particular face." << endl;
   	    }

    if(groundFace != oldGroundFace && groundFace != -1) {
	cout << "New face number, current ground face is: " << groundFace << endl;
        groundFaceHistory.push_back(groundFace);

	double currentDisplacement = displacement(subject);
        cout << "Absolute distance moved: " << currentDisplacement << endl;   
	
	/*if(saveData) {
            write2file(groundFaceHistory[0], "groundFace :", false);
            write2file(groundFaceHistory.back(), "", true);
	    write2file(groundFaceHistory[0], "Absolute distance moved :", false);
	    write2file(currentDisplacement, "", true);
        }*/
    } 
}

/* Function for writing data to file */
void T12Controller::write2file(double contentDouble, char const* contentString, bool isDouble) {
    
    ofstream myFile(path.c_str(), ios::app);
    if(isDouble) {
        myFile << contentDouble << endl;
    } else {
        myFile << contentString << endl;
    }
    myFile.close();

    cout << "Writing to file completed." << endl;
}

/* Get the file name for svaed data, name based on current date and time */
void T12Controller::getFileName(void) {

    time_t t = time(0);
    tm* now = localtime(&t);
 
    string temp = ".txt";
    ostringstream path_out(temp);

    path_out << "/home/hannah/Projects/NTRTsim/src/dev/hpetersson/12BarTensegrity/outputFiles/testnograph2";

    time_t year = (now->tm_year + 1900);
    time_t month = (now->tm_mon + 1);
    time_t day = (now->tm_mday);
    time_t hour = (now->tm_hour);
    time_t min = (now->tm_min);
    time_t sec = (now->tm_sec);

    path_out << "_" << year << "-" << "0" << month << "-" << day << "_" << hour << ":" << min << ":" << sec << ".txt";
    path = path_out.str();

}



void T12Controller::saveData2File(void) { 

    // Sine params
    // write2file(0, "Amplitude   -   Angular Frequency   -   Phase Change   -   DC Offset", 0); 
    for (int i = 0; i < nSquareClusters; i++) {
	cout << "i " << i << endl;
	write2file(i, "", 1);
	cout << "amplitude: " << amplitude[i] << endl;
	write2file(amplitude[i], "", 1);
    }
    // actions
    // ball COM distance
    // faces (inclusing hexa)

}


