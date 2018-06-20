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
 * @file T12ControllerGround.cpp
 * @brief Controller for T12 Model 
 * @author Hannah Petersson based on code from Steven Lessard
 * @version 1.0.0
 * $Id$
 */

// This module
#include "T12ControllerGround.h"
// This application
#include "T12ModelGround.h"
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
bool tweakParams = false; // When reading parameters from file, tweak with up to 0.5%

//Constructor using the model subject and a single pref length for all muscles.
//Currently calibrated to decimeters
T12ControllerGround::T12ControllerGround(T12ModelGround* subject, const double initialLength, double startTime) :
    m_initialLengths(initialLength),
    m_startTime(startTime),
    m_totalTime(0.0),
    maxStringLengthFactor(0.90),
    nSquareClusters(6),  // 6 = number of squares on 12Bar. On SUPERball, the number of faces is 8.
    nHexaClusters(8), 
    musclesPerSquareCluster(4), // 4 = number of muscles per square. On SUPERball, the number is 3.
    musclesPerHexaCluster(6)
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

/* Set the lengths of the muscles and initialize the learning adapter */
void T12ControllerGround::onSetup(T12ModelGround& subject)
{
    double dt = 0.0001;
    
    cout << "Current time is: " << m_totalTime << endl;

    //Set the initial length of every muscle in the subject
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
        tgBasicActuator * const pMuscle = muscles[i];
        assert(pMuscle != NULL);
        pMuscle->setControlInput(this->m_initialLengths, dt);
    }
   
    // Populate the clusters with muscles
    populateClusters(subject);

    initPosition = subject.getBallCOM();
    saveCOM(subject);

    // DEBUGGING
    /*cout << "initPosition x: " << initPosition[0] << endl;
    cout << "initPosition y: " << initPosition[1] << endl;
    cout << "initPosition z: " << initPosition[2] << endl;
    cout << endl;*/

    // If learning is used, setup adapter and learning parameters
    vector< vector<double> > actions;

    initializeRates(); // For muscle actuation

    //transform them to the size of the structure
    //actions = transformActions(actions);

    //apply these actions to the appropriate muscles according to the sensor values
    // (If parameters are read from file, this is done in transformActions)
    applyActions(subject, actions);

    cout << "Setup completed." << endl;
}

void T12ControllerGround::onStep(T12ModelGround& subject, double dt)
{
    int index = 0;
    //cout << "Current time: " << m_totalTime << endl;
    if (dt <= 0.0) {
        throw std::invalid_argument("dt is not positive");
    }
    m_totalTime+=dt;
    //cout << "Entered onStep." << endl;
    double newCluster, currentCluster, oldCluster;

    newCluster = getGroundFace(subject);
    if (index == 0) oldCluster = -1; 

    if( m_totalTime > m_startTime) {
        currentCluster = getGroundFace(subject); // check which face the robot is currently standing on 
        //cout << "Current cluster: " << currentCluster << endl;
	//while (currentCluster == newCluster) {
            setPreferredMuscleLengths(subject, dt, oldCluster, currentCluster); 
 
            const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    
            //Move motors for 
            /*for (size_t i = 0; i < muscles.size(); ++i)
            {
                tgBasicActuator * const pMuscle = muscles[i];
                assert(pMuscle != NULL);
                pMuscle->moveMotors(dt);
	        //cout << "Motor moved for muscle  " << pMuscle << endl;
            }*/
            newCluster = getGroundFace(subject);
	    if (newCluster != currentCluster) oldCluster == currentCluster; 
	 //  cout << "New cluster: " << newCluster << endl;
//}	//} }
    }
    index++;
}

// Things to be done in between each simulation, calculate scores, save data and clear parameters
void T12ControllerGround::onTeardown(T12ModelGround& subject) {
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
 * Returns the modified actions 2D vector such that 
 * each action value is now scaled to fit the model
 * Invariant: actions[x].size() == 4 for all legal values of x
 * Invariant: Each actions[] contains: amplitude, angularFrequency, phase, dcOffset
 */
vector< vector <double> > T12ControllerGround::transformActions(vector< vector <double> > actions1D)
{
    vector< vector <double> > actions2D(2, vector<double>(2)); // Vector to be returned

    // If reading parameters from file, do this
    /*if(!useLearning) { 
       vector <double> manualParams(24, 1); // '4' for the number of sine wave parameters, nClusters = 6 -> 24 total
        const char* filename = "/home/hannah/Projects/NTRTsim/src/dev/hpetersson/12BarTensegrity/InputActions/actions_11106.csv";
        std::cout << "Using manually set parameters from file " << filename << endl; 
        int lineNumber = 1;
        manualParams = readManualParams(lineNumber, filename);  
	for(int i = 0; i<squareClusters.size(); i++) { 
	    amplitude[i] = manualParams[i];
	    angularFrequency[i] = manualParams[i+squareClusters.size()];
	    phase[i] = manualParams[i+2*squareClusters.size()];
	    dcOffset[i] = manualParams[i+3*squareClusters.size()];
	}
        printSineParams();
    }*/

    // If learning is used, do this
    if(useLearning) {
//    double pretension = 0.9; // Tweak this value if need be. What is this actually?

         // Minimum rates
        double minRate  = 0.2; // dm/s 

        // Maximum amplitude, angularFrequency, phase, and dcOffset
        double maxRate = 1.5; // dm/s 

        assert(maxRate-minRate>0);
        double range = maxRate - minRate;

        // Apply output of learing to all rates
        int k = 0;
        for(int i=0;i<actions2D.size();i++) { //2x
            for (int j=0; j<actions2D[i].size(); j++) { //2x
                actions2D[i][j] = actions1D[i][j]*range + minRate;
                cout << "action2D: " << actions2D[i][j] << endl;
            }
        }
    }
   
    return actions2D;
}

/**
 * Defines the retract rate and elongation rate of the squares and hexagons according to actions
 */
void T12ControllerGround::applyActions(T12ModelGround& subject, vector< vector <double> > actions)
{
    // Apply actions by cluster
    retractRate[0] = 10; //actions[0][0]; // square    
    retractRate[1] = 10; //actions[1][0]; // hexagon
    elongateRate[0] = 10; //actions[0][1]; // square
    elongateRate[1] = 10; //actions[1][1]; // hexagon

    cout << "Actions applied. " << retractRate[0] << retractRate[1] << elongateRate[0] << elongateRate[1] << endl;
}



void T12ControllerGround::setPreferredMuscleLengths(T12ModelGround& subject, double dt, double oldCluster, double currentCluster) {

    vector<int> adjacentClusters = getAdjacentFace(currentCluster);
    int adjacentClusterForward = adjacentClusters[0];
    int adjacentClusterBackward = adjacentClusters[1];
    //cout << "The adjacent faces are: " << adjacentClusters[0] << " " << adjacentClusters[1] << endl;
//    assert(adjacentClusterForward != '\0');
//    assert(adjacentClusterBackward != NULL);

    vector<tgBasicActuator*> movedMuscle; 
 
    double newLength;
    double minLength = 0.1; // (Initial length = 1)
    double maxLength = 1.5;

    int usedIndex = 0;

    // FIRST: elongate all the muscles connected to the adjacent face in the backward direction to guide the robot to not roll over backwards
    if (adjacentClusterBackward < 6) { // Face is a square
	for (int i = 0; i < squareClusters[0].size(); i++) { 
	    usedIndex = 0;
	    tgBasicActuator *const pMuscle = squareClusters[adjacentClusterBackward][i];
	    assert(pMuscle != NULL);
	    //cout << "Muscle from to be retracted from squareCluster[" << currentCluster << "][" << i  << "] with ID: " << squareClusters[currentCluster][i] << endl;
	    double currentLength = pMuscle->getRestLength();
	    newLength = currentLength + elongateRate[0]*dt;
	    //cout << "Current Length: " << currentLength << " and new length: " << newLength << endl;
            if (newLength > minLength && newLength < maxLength) {
   	        //pMuscle->setControlInput(newLength);
   	        pMuscle->setControlInput(newLength, dt);
	        movedMuscle.push_back(pMuscle);
    	    }
//	    else //cout << "Adjacent square muscle wanted to be outside of boundaries" << endl;
        }	
    }

    else if (adjacentClusterBackward > 5) { // Ground face is a hexagon
	for (int i = 0; i < hexaClusters[0].size(); i++) { 
	    usedIndex = 0;
	    tgBasicActuator *const pMuscle = hexaClusters[adjacentClusterBackward - squareClusters.size()][i];
	    assert(pMuscle != NULL);
	    //cout << "Muscle from to be retracted from hexaCluster[" << currentCluster - squareClusters.size()<< "][" << i << "] with ID: " << hexaClusters[currentCluster - squareClusters.size()][i] << endl;
	    double currentLength = pMuscle->getRestLength();
	    newLength = currentLength + elongateRate[1]*dt;
	    //cout << "Current Length: " << currentLength << " and new length: " << newLength << endl;
            if (newLength > minLength && newLength < maxLength) {
   	        //pMuscle->setControlInput(newLength);
   	        pMuscle->setControlInput(newLength, dt);
	        movedMuscle.push_back(pMuscle);
    	    }// else cout << "Hexagon muscle wanted to be outside of boundaries." << endl;
	}
    }
/*
    cout << "Size of vector: " << movedMuscle.size() << endl;
    for (int i = 0; i < movedMuscle.size(); i++) {
	cout << "Muscle no " << i << " with ID " << movedMuscle[i] << endl;
    }*/   
    // SECOND: retract all the muscles connected to the adjacent face in the forward direction to guide the robot to roll over forward
    if (adjacentClusterForward < 6) { // Face is a square
	for (int i = 0; i < squareClusters[0].size(); i++) { 
	    usedIndex = 0;
	    tgBasicActuator *const pMuscle = squareClusters[adjacentClusterForward][i];
	    assert(pMuscle != NULL);
//	    cout << "Muscle from to be retracted from squareCluster[" << adjacentClusterForward << "][" << i  << "] with ID: " << squareClusters[adjacentClusterForward][i] << endl;
	    double currentLength = pMuscle->getRestLength();
	    newLength = currentLength - retractRate[0]*dt;
	    //cout << "Current Length: " << currentLength << " and new length: " << newLength << endl;

	    for (int b = 0; b < movedMuscle.size(); b++) {
//	 	cout << "b " << b << " movedMuslce " << movedMuscle[b] << " pMuscle " << pMuscle << endl;
		if (movedMuscle[b] == pMuscle) {
			usedIndex = 1;
//			cout << "usedIndex = " << usedIndex << " for muscle " << pMuscle << endl;
		}
	    }
            if (newLength > minLength && newLength < maxLength && usedIndex == 0) {
   	        pMuscle->setControlInput(newLength, dt);
	        movedMuscle.push_back(pMuscle);
   	        //pMuscle->setControlInput(newLength);
    	    }   else usedIndex = 0;
        }	
    }

    else if (adjacentClusterForward > 5) { // Ground face is a hexagon
	for (int i = 0; i < hexaClusters[0].size(); i++) { 
	    usedIndex = 0;
	    tgBasicActuator *const pMuscle = hexaClusters[adjacentClusterForward - squareClusters.size()][i];
	    assert(pMuscle != NULL);
	    //cout << "Muscle from to be retracted from hexaCluster[" << currentCluster - squareClusters.size()<< "][" << i << "] with ID: " << hexaClusters[currentCluster - squareClusters.size()][i] << endl;
	    double currentLength = pMuscle->getRestLength();
	    newLength = currentLength - retractRate[1]*dt;
//	    cout << "Current Length: " << currentLength << " and new length: " << newLength << endl;
        
	    for (int b = 0; b < movedMuscle.size(); b++) {
//	 	cout << "b " << b << endl;
		if (movedMuscle[b] == pMuscle) {
			usedIndex = 1;
//			cout << "usedIndex = " << usedIndex << " for muscle " << pMuscle << endl;
		}
	    }    
	    if (newLength > minLength && newLength < maxLength && usedIndex == 0) {
   	        pMuscle->setControlInput(newLength, dt);
	        movedMuscle.push_back(pMuscle);
   	        //pMuscle->setControlInput(newLength);
    	    } else usedIndex = 0;
//else cout << "Hexagon muscle wanted to be outside of boundaries." << endl;
	}
    }
  /*  cout << "Size of vector: " << movedMuscle.size() << endl;
    for (int i = 0; i < movedMuscle.size(); i++) {
	cout << "Muscle no " << i << " with ID " << movedMuscle[i] << endl;
    }*/

// THIRD: retract all the muscles connected to the ground face to minimize the area the robot is standing on
    if (currentCluster < 6) { // Ground face is a square
	for (int i = 0; i < squareClusters[0].size(); i++) { 
	    usedIndex = 0;
	    tgBasicActuator *const pMuscle = squareClusters[currentCluster][i];
	    assert(pMuscle != NULL);
	    //cout << "Muscle from to be retracted from squareCluster[" << currentCluster << "][" << i  << "] with ID: " << squareClusters[currentCluster][i] << endl;
	    double currentLength = pMuscle->getRestLength();
	    newLength = currentLength - retractRate[0]*dt;
	    //cout << "Current Length: " << currentLength << " and new length: " << newLength << endl;
         
	    for (int b = 0; b < movedMuscle.size(); b++) {
	 //	cout << "b " << b << endl;
		if (movedMuscle[b] == pMuscle) {
			usedIndex = 1;
//			cout << "usedIndex = " << usedIndex << " for muscle " << pMuscle << endl;
		}
	    } 
            if (newLength > minLength && newLength < maxLength && usedIndex == 0) {
   	        pMuscle->setControlInput(newLength, dt);
	        movedMuscle.push_back(pMuscle);
   	        //pMuscle->setControlInput(newLength);
    	    } else usedIndex = 0;

	    //else cout << "Square muscle wanted to be outside of boundaries" << endl;
        }	
    }

    else if (currentCluster > 5) { // Ground face is a hexagon
	for (int i = 0; i < hexaClusters[0].size(); i++) { 
	    usedIndex = 0;
	    tgBasicActuator *const pMuscle = hexaClusters[currentCluster - squareClusters.size()][i];
	    assert(pMuscle != NULL);
//	    cout << "Muscle from to be retracted from hexaCluster[" << currentCluster - squareClusters.size()<< "][" << i << "] with ID: " << hexaClusters[currentCluster - squareClusters.size()][i] << endl;
	    double currentLength = pMuscle->getRestLength();
	    newLength = currentLength - retractRate[1]*dt;
	    //cout << "Current Length: " << currentLength << " and new length: " << newLength << endl;
     
	    for (int b = 0; b < movedMuscle.size(); b++) {
	 //	cout << "b " << b << endl;
		if (movedMuscle[b] == pMuscle) {
			usedIndex = 1;
//			cout << "usedIndex = " << usedIndex << " for muscle " << pMuscle << endl;
		}
	    }           
	    if (newLength > minLength && newLength < maxLength && usedIndex == 0) {
   	        //pMuscle->setControlInput(newLength);
   	        pMuscle->setControlInput(newLength, dt);
	        movedMuscle.push_back(pMuscle);
    	    } else usedIndex = 0;
//else cout << "Hexagon muscle wanted to be outside of boundaries." << endl;
	}
    }



    /*cout << "Size of vector: " << movedMuscle.size() << endl;
    for (int i = 0; i < movedMuscle.size(); i++) {
	cout << "Muscle no " << i << " with ID " << movedMuscle[i] << endl;
    }
  */ 
 // FOURTH: check all muscles to return the previously used ones to the initial length
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    for (int i = 0; i < muscles.size(); i++) { 
	usedIndex = 0;
        tgBasicActuator * const pMuscle = muscles[i];
	assert(pMuscle != NULL); 
	double currentLength = pMuscle->getRestLength();
	for (int b = 0; b < movedMuscle.size(); b++) {
	 //	cout << "b " << b << endl;
	    if (movedMuscle[b] == pMuscle) {
		usedIndex = 1;
//		cout << "usedIndex = " << usedIndex << " for muscle " << pMuscle << endl;
	    }
	}           
	if (currentLength > m_initialLengths) newLength = currentLength - retractRate[0]*dt; // implement +/-5%   
        else if (currentLength < m_initialLengths) newLength = currentLength + elongateRate[0]*dt;
	else newLength = currentLength;
	if (usedIndex == 0) {
	    pMuscle->setControlInput(newLength, dt);
	    movedMuscle.push_back(pMuscle);
	}
//	cout << "All muscles altered." << endl;
    }





    /*cout << "Size of vector: " << movedMuscle.size() << endl;
    for (int i = 0; i < movedMuscle.size(); i++) {
	cout << "Muscle no " << i << " with ID " << movedMuscle[i] << endl;
    }*/
//while(1);    
    // Elongate old clusters muscles to the start length. This action will override the previous control input
    // in case one muscle is in both the old and new cluster. The muscle should in this case be elongated, 
    // to not make the robot fall back but move forward. 
 /*   if(oldClusterSquare && oldCluster != -1) {
	for (int i = 0; i < squareClusters[0].size(); i++) { 
	    tgBasicActuator *const pMuscle = squareClusters[oldCluster][i];
	    assert(pMuscle != NULL);
	    //cout << "Muscle to be elongated from squareCluster[" << oldCluster << "][" << i << "] with ID: " << squareClusters[oldCluster][i];
	    double currentLength = pMuscle->getRestLength();
            if (currentLength < m_initialLengths) {
		newLength = currentLength + elongateRate[0]*dt;
		pMuscle->setControlInput(newLength, dt);
    		}
	}	
    }
   
    if(!oldClusterSquare && oldCluster != -1) {
	for (int i = 0; i < hexaClusters[0].size(); i++) { 
	    tgBasicActuator *const pMuscle = hexaClusters[oldCluster - squareClusters.size()][i];
	    assert(pMuscle != NULL);
	    //cout << "Muscle to be elongated from hexaCluster[" << i << "][" << oldCluster - squareClusters.size() << "] with ID: " << hexaClusters[oldCluster - squareClusters.size()][i] << endl;
	    double currentLength = pMuscle->getRestLength();
            if (currentLength < m_initialLengths) {
		newLength = currentLength + elongateRate[1]*dt;
		pMuscle->setControlInput(newLength, dt);
    		}
	}	
    }


	 

*/


    // Calculate the new length, one cluster at a time, and apply it to that cluster
/*    for(int j = 0; j < squareClusters.size(); j++) { // squareClusters.size() gives number of columns
	double newLength = 0.7 * sin(1.88 * m_totalTime + 2.8) + 1;
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
	    
        }*/ 
     
     // DEBUGGING
    /*cout << "Square cluster: " << endl;
    for(int j=0; j<6; j++) {
	for(int i=0; i<4; i++){
		cout << squareClusters[i][j] << " ";
	}
        cout << endl;
    }*/
    // } 
}

void T12ControllerGround::populateClusters(T12ModelGround& subject) {
    int nMuscles = 36;
    int iMuscle = 0;
  
    vector <tgBasicActuator*> squareCol; // Vectors to create cluster matrices
    //squareClusters.push_back(squareCol);
    vector <tgBasicActuator*> hexaCol;  
    // hexaClusters.push_back(hexaCol);

    int i0 = 0, i1 = 0, i2 = 0, i3 = 0, i4 = 0, i5 = 0, i6 =0, i7 = 0, i8 = 0, i9 = 0, i10 = 0, i11 = 0, i12 = 0, i13 = 0;


    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles(); // Save all muscles in vector

    // Populate square clusters
    for(iMuscle = 0; iMuscle < nMuscles; iMuscle++) {
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
        } else {
    //        cout << "Muscle is not related to the square faces." << endl;
        }

        // Group muscles in clusters, hexagon
        if (iMuscle == 0 || iMuscle == 1|| iMuscle == 22 || iMuscle == 23 || iMuscle == 30 || iMuscle == 35) { // Cluster 6
            hexaClusters[0][i6] = pMuscle;
            i6++;
        } if (iMuscle == 0 || iMuscle == 2 || iMuscle == 6 || iMuscle == 8 || iMuscle == 12 || iMuscle == 14) { // Cluster 7
            hexaClusters[1][i7] = pMuscle;
            i7++;
        } if (iMuscle == 4 || iMuscle == 5 || iMuscle == 9 || iMuscle == 10 || iMuscle == 15 || iMuscle == 17) { // Cluster 8
            hexaClusters[2][i8] = pMuscle;
            i8++;
        } if (iMuscle == 3 || iMuscle == 4 || iMuscle == 18 || iMuscle == 19 || iMuscle == 22 || iMuscle == 26) { // Cluster 9
            hexaClusters[3][i9] = pMuscle;
            i9++;
        } if (iMuscle == 6 || iMuscle == 7 || iMuscle == 21 || iMuscle == 24 || iMuscle == 27 || iMuscle == 29) { // Cluster 10
            hexaClusters[4][i10] = pMuscle;
            i10++;
        } if (iMuscle == 13 || iMuscle == 14 || iMuscle == 19 || iMuscle == 20 || iMuscle == 32 || iMuscle == 33) { // Cluster 11
            hexaClusters[5][i11] = pMuscle;
            i11++;
        } if (iMuscle == 16 || iMuscle == 17 || iMuscle == 27 || iMuscle == 28 || iMuscle == 31 || iMuscle == 33) { // Cluster 12
            hexaClusters[6][i12] = pMuscle;
            i12++;
        } if (iMuscle == 10 || iMuscle == 11 || iMuscle == 24 || iMuscle == 25 || iMuscle == 34 || iMuscle == 35) { // Cluster 13
            hexaClusters[7][i13] = pMuscle;
            i13++;
        } else {
    //        cout << "Muscle is not related to the square faces." << endl;
        }
    }

    // Make sure it is ok
    cout << "Square clusters: " << endl;
    for(int i = 0; i<squareClusters.size(); i++) {
	for(int j = 0; j<squareClusters[0].size(); j++) {
	    cout << squareClusters[i][j] << " "; 
	}
	cout << endl;
    }
    cout << "Hexa clusters: " << endl;
    for(int i = 0; i<hexaClusters.size(); i++) {
	for(int j = 0; j<hexaClusters[0].size(); j++) {
	    cout << hexaClusters[i][j] << " "; 
	}
	cout << endl;
    }
}

/* Initializes sine waves, each cluster has identical parameters */
void T12ControllerGround::initializeRates() {
    cout << "Rates initialized." << endl;
    retractRate = new double[2]; // [0] for squares, [1] for hexagons
    elongateRate = new double[2];
    // DEBUGGING
//    cout << "Rates initialized. Sizes: " << retractRate.size() << elongateRate.size()  << endl;
}

double T12ControllerGround::displacement(T12ModelGround& subject) {
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

std::vector<double> T12ControllerGround::readManualParams(int lineNumber, const char* filename) {
    assert(lineNumber > 0);
    vector<double> result(24, 1.0); // earlier, this was set to 32. I don\t see the reason why 
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

    // Tweak each read-in parameter by as much as 0.5% (params range: [0,1])     
    if (tweakParams) {
        cout << "Tweaking parameters from file with up to 0.5%." << endl;
        for (int i=0; i < result.size(); i++) {
            //std::cout<<"Cell " << i << ": " << result[i] << "\n";
            double seed = ((double) (rand() % 100)) / 100;
            result[i] += (0.01 * seed) - 0.005; // Value +/- 0.005 of original

            //if(result[i] >= 1) {
             //   result[i] = result[i] - 0.5; // Dummy solution before values from learning is found for 12 bar
            //}
        }
    }

    return result;
}

void T12ControllerGround::printRates() {
    cout << endl;
    cout << "Square retract rate: " << retractRate[0] << endl;
    cout << "Hexagon retract rate: " << retractRate[1] << endl;
    cout << "Square elongation rate: " << elongateRate[0] << endl;
    cout << "Hexagon elongation rate: " << elongateRate[1] << endl;
    cout << endl; 
}

// @ TODO: Implement for hexagon faces
// For a certain rod configuration, determine corresponding face 
void T12ControllerGround::determineFace(bool isSquareFace) {

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
double T12ControllerGround::totalEnergySpent(T12ModelGround& subject) {

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
double T12ControllerGround::getGroundFace(T12ModelGround& subject) {

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
    distanceMovedSnorkel += abs(oldManhattan - distanceMovedManhattan); 
    if(groundFace != oldGroundFace) { // && groundFace != -1) {
        groundFaceHistory.push_back(groundFace); // Save ground face in history log
	saveCOM(subject);
    } 

    return groundFace;
}


void T12ControllerGround::saveCOM(T12ModelGround& subject) {

    vector<double> COM = subject.getBallCOM();
    COMx.push_back(COM[0]);
    COMy.push_back(COM[1]);
    COMz.push_back(COM[2]);

    cout << "COM: " << COM[0] << " " << COM[1] << " " << COM[2] << endl;
}

vector<int> T12ControllerGround::getAdjacentFace(double groundFace) {

    vector<int> adjacentFace(2);

//    cout << "ground face: " << groundFace << endl;
    if (groundFace == 7) {
        adjacentFace[0] = 0; 
  	adjacentFace[1] = 10;
    } else if (groundFace == 0) { 
	adjacentFace[0] = 9;
	adjacentFace[1] = 7;
    } else if (groundFace == 9) {
	adjacentFace[0] = 8;
	adjacentFace[1] = 0;
    } else if (groundFace == 8) {
	adjacentFace[0] = 3;
	adjacentFace[1] = 9;
    } else if (groundFace == 3) {
	adjacentFace[0] = 10;
	adjacentFace[1] = 8;
    } else if (groundFace == 10) {
	adjacentFace[0] = 7;
	adjacentFace[1] = 3;
    }

    //cout << "adjacent face: " << adjacentFace[0] << adjacentFace[1] << endl;
    return adjacentFace; 

}

/* Function for writing data to txt file */
void T12ControllerGround::write2txtFile(double contentDouble, char const* contentString, bool isDouble) {
    
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
void T12ControllerGround::write2csvFile(double contentDouble, char const* sign, bool isDouble) {
    
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
void T12ControllerGround::getFileName(void) {

    time_t t = time(0);
    tm* now = localtime(&t);
 
    string txttemp = ".txt";
    string csvtemp = ".csv";

    ostringstream txt_path_out(txttemp);
    ostringstream csv_path_out(csvtemp);

    txt_path_out << "/home/hannah/Projects/NTRTsim/src/dev/hpetersson/ground12BarTensegrity/outputFiles/textgen_z_";
    csv_path_out << "/home/hannah/Projects/NTRTsim/src/dev/hpetersson/ground12BarTensegrity/outputFiles/gen_z_";

    
    time_t year = (now->tm_year + 1900);
    time_t month = (now->tm_mon + 1);
    time_t day = (now->tm_mday);
    time_t hour = (now->tm_hour);
    time_t min = (now->tm_min);
    time_t sec = (now->tm_sec);

    txt_path_out << year << "0" << month << day << "_" << hour << "h" << min << "m" << sec << "s" << ".txt";
    csv_path_out << year << "0" << month << day << "_" << hour << "h" << min << "m" << sec << "s" << ".csv";
 
    txtPath = txt_path_out.str();
    csvPath = csv_path_out.str();
    cout << "File name is " << csvPath << endl;
    // Print header for csv file
    write2csvFile(0, "Simulation number,Manhattan distance,Snirky distance,Energy spent,Initial Length,Start time,Total time,Faces,COMx,COMy,COMz", 0);
    write2csvFile(0, "\n", 0);
    cout << "File name is " << csvPath << endl;
}



void T12ControllerGround::saveData2File(void) { 

// WRITE TO TEXT FILE FOR EASY ACCESS TO DATA
    // Sine params
   /* write2txtFile(0, "Index - Amplitude - Angular Frequency - Phase Change - DC Offset", 0); 
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
    }*/

    write2txtFile(0, "\n", 0);
    
    // Actions
    /*write2txtFile(0, "Actions", 0); 
    write2txtFile(0, "\n", 0);
    for (int i = 0; i < actions[0].size(); i++) {
	for (int j = 0; j < actions.size(); j++) {  
            write2txtFile(actions[j][i], "", 1);
	    write2txtFile(0, ", ", 0);
	}
	write2txtFile(0, "\n", 0);
    }*/

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
    cout << "Total distance moved: " << distanceMovedSnorkel << endl; 
    write2csvFile(distanceMovedSnorkel,"",1);
    write2csvFile(0,",",0);

    // energy spent 
    write2csvFile(energySpent,"",1);
    write2csvFile(0,",",0);

    // Initial length
    write2csvFile(m_initialLengths, "", 1);
    write2csvFile(0, ",", 0);
    

    // Start time
    write2csvFile(m_startTime, "", 1);
    write2csvFile(0, ",", 0);

    // Total time
    write2csvFile(m_totalTime, "", 1);
    write2csvFile(0, ",", 0);



    // ground face
    for(int i = 0; i < groundFaceHistory.size(); i++) {
        write2csvFile(groundFaceHistory[i], "", 1);
 	write2csvFile(0, ",", 0);
    }

    write2csvFile(m_totalTime, "", 1);
    write2csvFile(0, ",", 0);
    
    write2csvFile(0, "NaN", 0);
    write2csvFile(0, ",", 0);

    // COM
    for (int i = 0; i < COMx.size(); i++)  { // x
        write2csvFile(COMx[i],"",1);
	write2csvFile(0,",", 0);
    }
    write2csvFile(0, "NaN", 0);
    write2csvFile(0, ",", 0);


    for (int i = 0; i < COMy.size(); i++)  { // y
        write2csvFile(COMy[i],"",1);
	write2csvFile(0,",", 0);
    }
    write2csvFile(0, "NaN", 0);
    write2csvFile(0, ",", 0);

 
    for (int i = 0; i < COMz.size(); i++)  { // z
        write2csvFile(COMz[i],"",1);
	write2csvFile(0,",", 0);
    }
    write2csvFile(0, "NaN", 0);   
    write2csvFile(0, ",", 0);

    write2csvFile(0, "NaN", 0);   
    write2csvFile(0, "\n", 0);

}

void T12ControllerGround::clearParams(void) { 
   
    cout << "Current time is: " << m_totalTime << endl;
    groundFaceHistory.clear();
    distanceMovedManhattan = 0;
    distanceMovedSnorkel = 0;
    energySpent = 0;
    m_totalTime = 0; 
    groundFace = 0;
    initPosition.clear();

    for (int i = 0; i < 2; i++) {
	retractRate[i] = 0;
	elongateRate[i] = 0;
    }

    actions.clear();

}
