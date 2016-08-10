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
 * @file TensegrityModelController.cpp
 * @brief Implementation of TensegrityModelController.
 * @author Drew Sabelhaus, Lara Janse van Vuuren
 * $Id$
 */

// This module
#include "TensegrityModelController.h"
// This application
#include "TensegrityModel.h"
// This library
//#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"

//#include "sensors/tgDataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "helpers/FileHelpers.h"

//Constructor does nothing at the moment
TensegrityModelController::TensegrityModelController()
{
}

//Debugging output interaction with yaml
void TensegrityModelController::onSetup(TensegrityModel& subject)
{
	std::cout << "Setting up the tensegrity model controller" << std::endl;
	//Tags from entire structure
        const tgTags AllTags = subject.getTags(); 
	//Get array of strings
	const std::deque<std::string>& AllTagStringsArray = AllTags.getTags();
 	//Iterate through array and output strings to command line
	for (std::size_t i = 0; i < AllTagStringsArray.size(); i ++) 
	{	
		std::cout << AllTagStringsArray[i] << std::endl;
	}   
   	std::cout << "Finished outputting tags" << std::endl;    
	//Outputting entire model
	std::cout << "Contents of model: " << std::endl;
	std::cout << subject << std::endl;
}

void TensegrityModelController::onStep(TensegrityModel& subject, double dt)
{
}
	
 
