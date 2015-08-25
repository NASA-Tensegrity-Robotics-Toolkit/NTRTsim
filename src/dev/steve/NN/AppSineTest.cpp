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
 * @file SineTest.cpp
 * $Id$
 */

#include "neuralNet/Neural Network v2/neuralNetwork.h"
#include "neuralNet/Neural Network v2/neuralNetworkTrainer.h"
#include <string>
#include <fstream>
#include <sstream>
// The C++ Standard Library
#include <iostream>

void prepareNN();

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv) {
    std::cout << "SineTest" << std::endl;

    // Prepare NN
    prepareNN();
}
 
void prepareNN() {
    //seed random number generator
    srand( (unsigned int) time(0) );

    //create data set reader and load data file
    dataReader d;
    d.loadDataFile("letter-recognition-2.csv",16,3);
    d.setCreationApproach( STATIC, 10 );    

    //create neural network
    neuralNetwork nn(16,10,3);

    //create neural network trainer
    neuralNetworkTrainer nT( &nn );
    nT.setTrainingParameters(0.001, 0.9, false);
    nT.setStoppingConditions(150, 90);
    nT.enableLogging("log.csv", 5); 

    //train neural network on data sets
    for (int i=0; i < d.getNumTrainingSets(); i++ )
    {   
        nT.trainNetwork( d.getTrainingDataSet() );
    }   

    //save the weights
    nn.saveWeights("weights.csv");

    std::cout << std::endl << std::endl << "-- END OF PROGRAM --" << std::endl;
    char c; std::cin >> c;
}    

