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
 * @file FileHelpers.cpp
 * @brief Contains helper methods for file I/O. 
 * $Id$
 */

#include <string>
#include <fstream>
#include <sstream>
#include "FileHelpers.h"
#include "resources.h"

using namespace std;

std::string FileHelpers::getFileString(std::string fileName) {
    std::ifstream fileInput(fileName.c_str());
    stringstream buffer;
    buffer << fileInput.rdbuf();
    return buffer.str();
}

std::string FileHelpers::getResourcePath(std::string relPath) {
    stringstream buffer;
    buffer << RESOURCE_PATH << "/" << relPath;
    return buffer.str();
}

std::string FileHelpers::getTestResourcePath(std::string relPath) {
    stringstream buffer;
    buffer << TEST_RESOURCE_PATH << "/" << relPath;
    return buffer.str();
}

double FileHelpers::getFinalScore(std::string filePath)
{
	ifstream results;
	results.open(filePath.c_str(), ios::in);
	
	std::string line;
	// Return the last non-empty line
	while (results >> std::ws && getline (results,line));;
	
	std::stringstream stream(line);
	
	double dist;
	
	// Get the first double from that line
	stream >> dist;
	
	results.close();
	
	return dist;
}
