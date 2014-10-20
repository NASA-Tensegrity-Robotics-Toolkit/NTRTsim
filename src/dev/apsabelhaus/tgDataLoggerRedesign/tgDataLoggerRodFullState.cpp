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
 * @file tgDataLoggerRodFullState.cpp
 * @brief Contains the implementation of class tgDataLoggerRodFullState
 * @author Drew Sabelhaus and Brian Tietz
 * $Id$
 */

#include "tgDataLoggerRodFullState.h"

#include "core/tgRod.h"
#include "core/tgCast.h"
#include "core/tgTags.h"

#include <iostream>
#include <fstream>
#include <string>

/**
 * Construct our data logger
 */
tgDataLoggerRodFullState::tgDataLoggerRodFullState()
{
  numRods = 0;
}

/** Virtual base classes must have a virtual destructor. */
tgDataLoggerRodFullState::~tgDataLoggerRodFullState()
{ }

/**
 * Set our filename for output.
 */
void tgDataLoggerRodFullState::setFileName(std::string fileName)
{
  m_fileName = fileName;
}

// Check if the tgModel passed in is actually the type of object we
// want to log.
// TODO: make the string "tgRod" a const string or something somewhere,
// so we only have to set it once as opposed to multiple places
// for these various casts.
bool tgDataLoggerRodFullState::isThisMyLoggable(const tgModel* obj) const
{
    bool result;
    if(tgCast::cast<tgModel, tgRod>(obj) == 0)
    {
        result = false;
    }
    else
    {
	result =  true;
    }
    return result;
}


void tgDataLoggerRodFullState::writeHeader(tgModel* obj)
{
  // TODO: check if m_fileName has been set, throw exception if not
  // First, check if we're on the right object
  if( tgDataLoggerRodFullState::isThisMyLoggable(obj) )
  {
      // Then, generate the header string for this individual rod,
      // and write to file.
      // TODO: add tags here.
      std::ofstream tgOutput;
      tgOutput.open(m_fileName.c_str(), std::ios::app);
      std::stringstream rodName;
      // Add the rod number (count) to this rod's name
      rodName << "rod" << numRods;
      // Header here is X, Y, Z, Yaw, Pitch, Roll, mass
      // TODO: check what coordinate frame is meant with yaw, pitch, roll
      tgOutput << rodName.str() << "_X" << ","
	     << rodName.str() << "_Y" << ","
	     << rodName.str() << "_Z" << ","
	     << rodName.str() << "_Yaw" << ","
	     << rodName.str() << "_Pitch" << ","
	     << rodName.str() << "_Roll" << ","
	     << rodName.str() << "_mass" << ",";
      numRods++;
      tgOutput.close();
  }
}

void tgDataLoggerRodFullState::render(const tgModel& model) const
{
  // TODO: check if m_fileName has been set, throw exception if not
  // First, check if we're on the right object
  if( isThisMyLoggable(&model) )
  {
      // Then, open our output stream and render!
      std::ofstream tgOutput;
      tgOutput.open(m_fileName.c_str(), std::ios::app);
    
      // note that we need to cast here in order to access the actual methods
      // within tgRod.
      const tgRod* castedRod = tgCast::cast<tgModel, tgRod>(&model);
      btVector3 com = castedRod->centerOfMass();
      btVector3 angles = castedRod->orientation();
    
      tgOutput << com[0] << ","
	       << com[1] << ","
	       << com[2] << ","
	       << angles[0] << ","
	       << angles[1] << ","
	       << angles[2] << ","	
	       << castedRod->mass() << ",";

      tgOutput.close();
  }
}

