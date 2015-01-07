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
 * @file AppGATests.cpp
 * @brief An applicaiton to test the efficiency of various genetic algorithms
 * @author Brian Mirletz
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

// This application

// This library
#include "learning/NeuroEvolution/NeuroEvolution.h"
#include "learning/Adapters/NeuroAdapter.h"
#include "helpers/FileHelpers.h"
// The C++ Standard Library
#include <iostream>
#include <exception>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name; argv[1], if supplied, is the
 * suffix for the controller
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "AppGATests" << std::endl;
    
    /* Required for setting up learning file input/output. */
    const std::string suffix((argc > 1) ? argv[1] : "default");
    
    std::string fileName = "Config.ini";
    std::string path = "bmirletz/GATests/";
    
    NeuroEvolution testEvolution(suffix, fileName, path);
    
    std::string fullPath = FileHelpers::getResourcePath(path);
    
    configuration testConfigData;
    
    testConfigData.readFile(fullPath + fileName);
    bool learning = testConfigData.getintvalue("learning");
    
    NeuroAdapter testAdapter;
    
    int numberOfInputs = testConfigData.getintvalue("numberOfStates");
    int numberOfOutputs = testConfigData.getintvalue("numberOfActions");
    std::vector<double> state(numberOfInputs, 1.0);
    double goal = 1.0 * (double) numberOfOutputs;

    
    int steps = 0;
    while (steps < 30000)
    {
        testAdapter.initialize(&testEvolution,
                        learning,
                        testConfigData);
        
        std::vector<std::vector<double> > actions = testAdapter.step(0.0, state);
        double score1 = 0.0;
        for(std::size_t i = 0; i < actions.size(); i++)
        {
            for(std::size_t  j = 0; j < actions[i].size(); j++)
            {
                score1 += actions[i][j];
            }
        }
        
        std::vector<double> scores;
        scores.push_back(score1);
        scores.push_back(0.0);
        
        testAdapter.endEpisode(scores);
        
        if (score1 == goal)
        {
            break;
        }
        
        steps++;
    }
    
    //Teardown is handled by delete, so that should be automatic
    return 0;
}
