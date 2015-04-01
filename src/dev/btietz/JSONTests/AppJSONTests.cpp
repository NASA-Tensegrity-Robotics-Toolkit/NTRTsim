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
 * @file AppJSONTests.cpp
 * @brief An applicaiton to test some aspects of JSON configuration
 * @author Brian Mirletz
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

// This application

// This library
#include "helpers/FileHelpers.h"
// JSON
#include <json/json.h>
#include <json/value.h>
// The C++ Standard Library
#include <iostream>
#include <exception>
#include <vector>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name; argv[1], if supplied, is the
 * suffix for the controller
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "AppJSONTests" << std::endl;
    
    //BEGIN DESERIALIZING

    Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;

    bool parsingSuccessful = reader.parse( FileHelpers::getFileString("controlVars.json"), root );
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout << "Failed to parse configuration\n"
            << reader.getFormattedErrorMessages();
        /// @todo should this throw an exception instead??
    }
    // Get the value of the member of root named 'encoding', return 'UTF-8' if there is no
    // such member.
    Json::Value CPGVals = root.get("CPGVals", "UTF-8");
    
    std::cout << CPGVals.size() << " " << CPGVals[0].size()<< std::endl;
    
    Json::Value::iterator CPGIt = CPGVals.begin();
    
    std::vector<double> CPGVect;
    
    for(CPGIt = CPGVals.begin(); CPGIt != CPGVals.end(); CPGIt++)
    {
        std::cout << *CPGIt << std::endl;
        CPGVect.push_back((*CPGIt)[0].asDouble());
    }
    
    std::cout << CPGVals << std::endl;
    
    for(std::size_t i = 0; i < CPGVect.size(); i++)
    {
        std::cout << CPGVect[i] << " ";
    }
    
    std::cout << std::endl;
    
    //Teardown is handled by delete, so that should be automatic
    return 0;
}
