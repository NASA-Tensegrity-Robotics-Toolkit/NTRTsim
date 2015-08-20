/*
 * Copyright © 2015, United States Government, as represented by the
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
 * @file UpperLimbController.cpp
 * @brief Preferred Length Controller for UpperLimbModel
 * @author Steven Lessard
 * @version 1.0.0
 * $Id$
 */

// This module
#include "UpperLimbController.h"
// This application
#include "UpperLimbModel.h"
// This library
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <math.h>
#include <stdexcept>
#include <vector>

# define PI 3.14159265358979323846 
# define E 2.71828182845904523536

using namespace std;

UpperLimbController::UpperLimbController(const double initialLength, double timestep) :
    m_initialLengths(initialLength),
    m_totalTime(0.0),
    dt(timestep) 
{}

//Fetch all the muscles and set their preferred length
void UpperLimbController::onSetup(UpperLimbModel& subject) {
    initializeNeuralNet(subject);
    initializeMusclePretensions(subject);
	this->m_totalTime=0.0;
}

// Set target length of each muscle via NN, then move motors accordingly
void UpperLimbController::onStep(UpperLimbModel& subject, double dt) {
    // Update controller's internal time
    if (dt <= 0.0) { throw std::invalid_argument("dt is not positive"); }
    m_totalTime+=dt;

    populateOutputLayer();
    setTargetLengths(subject, dt);
    moveAllMotors(subject, dt);
}

/**
 * Create a Neural Net for the controller with a single, fully connected hidden layer
 * that links end-effector position (inputs) to target lengths for muscles (outputs)
 * Initialize all weights as values [0,1] randomly and uniformly 
 */
void UpperLimbController::initializeNeuralNet(UpperLimbModel& subject) {
    this->nInputNeurons = 3; //(x, y, z) of end-effector TODO: add velocity
    this->nHiddenNeurons = 10;
    this->nOutputNeurons = subject.getAllMuscles().size();
    this->nWeightsInput = nInputNeurons * nHiddenNeurons;
    this->nWeightsOutput = nOutputNeurons * nHiddenNeurons;
    this->inputLayer.resize(nInputNeurons);
    this->hiddenLayer.resize(nHiddenNeurons);
    this->outputLayer.resize(nOutputNeurons);
    initializeNeuralNetWeights();
}

// Import random values [0,1] to set as weights in the NN
void UpperLimbController::initializeNeuralNetWeights() {
    weights.resize(2); // 1+nHiddenLayers
    weights[0].resize(nWeightsInput);
    weights[1].resize(nWeightsOutput);
    importWeights();
}

// Import (nWeightsInput + nWeightsOutput) values to be used in the NN
void UpperLimbController::importWeights() {
    std::ifstream f("weights.dat", std::ifstream::in);

    if (f.is_open()) {
        std::cout << "'weights.dat' opened'" << std::endl;
        std::string s;
        double w = 0;

        for (size_t i=0; getline(f, s); i++) {
            w = atof(s.c_str());
            if(i<nWeightsInput) {
                weights[0][i] = w;
            } else {
                weights[1][i-nWeightsInput] = w;
            }
        }
        f.close();
    } else {
        std::cerr << "ERROR: Weight parameters file could not be opened" << std::endl;
        exit(1);
    }
}

void UpperLimbController::initializeMusclePretensions(UpperLimbModel& subject) {
    const double olecranonfascia_length = 4;
    const double brachioradialis_length = 12;
    const double anconeus_length        = 6;

	const std::vector<tgBasicActuator*> olecranonfascia = subject.find<tgBasicActuator>("olecranon");
	const std::vector<tgBasicActuator*> anconeus        = subject.find<tgBasicActuator>("anconeus");
	const std::vector<tgBasicActuator*> brachioradialis = subject.find<tgBasicActuator>("brachioradialis");

    for (size_t i=0; i<olecranonfascia.size(); i++) {
		tgBasicActuator * const pMuscle = olecranonfascia[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(olecranonfascia_length, dt);
    }
                                        
    // using for loops to anticipate more muscle fibers in the future
    for (size_t i=0; i<anconeus.size(); i++) {
		tgBasicActuator * const pMuscle = anconeus[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(anconeus_length, dt);
    }
     
    for (size_t i=0; i<brachioradialis.size(); i++) {
		tgBasicActuator * const pMuscle = brachioradialis[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(brachioradialis_length, dt);
    }
}

// Populate outputLayer by feeding the inputLayer through the NN
void UpperLimbController::populateOutputLayer() {
    double sum = 0;
    double x = 0.25; //TODO: make x meaningful odometry data

    // Sense end effector position for the input layer
    for (size_t i=0; i<nInputNeurons; i++) {
        inputLayer[i] = x;
    }
    // Populate hidden layer neurons
    for (size_t j=0; j<nHiddenNeurons; j++) {
        sum = 0;
        for (size_t i=0; i<nInputNeurons; i++) {
            sum += inputLayer[i] * weights[0][i*nHiddenNeurons + j];
        }
        hiddenLayer[j] = sigmoid(sum);
    }
     
    // Populate output layer neurons
    for (size_t k=0; k<nOutputNeurons; k++) {
        sum = 0;
        for (size_t j=0; j<nHiddenNeurons; j++) {
            sum += hiddenLayer[j] * weights[1][j*nOutputNeurons + k];
        }
        outputLayer[k] = sigmoid(sum);
    }
}

void UpperLimbController::setTargetLengths(UpperLimbModel& subject, double dt) {
    const std::vector<tgBasicActuator*> allMuscles = subject.getAllMuscles();
    double newLength = 0;

    for (size_t i=0; i<allMuscles.size(); i++) {
        //newLength = scale * outputLayer[i];
        newLength = 5;
        tgBasicActuator* const pMuscle = allMuscles[i]; 
		assert(pMuscle != NULL);
		pMuscle->setControlInput(newLength, dt);
    }
}

//Move motors for all the muscles
void UpperLimbController::moveAllMotors(UpperLimbModel& subject, double dt) {
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->moveMotors(dt);
	}
     
}

double UpperLimbController::sigmoid(double x) {
    return 1 / (1 + pow(E, -x));
}

