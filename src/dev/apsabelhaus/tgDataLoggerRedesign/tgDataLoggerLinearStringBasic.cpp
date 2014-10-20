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
 * @file tgDataLoggerLinearStringBasic.cpp
 * @brief Contains the implementation of class tgDataLoggerLinearStringBasic.
 * @author Drew Sabelhaus and Brian Tietz
 * $Id$
 */

#include "tgDataLoggerLinearStringBasic.h"

#include "core/tgLinearString.h"
#include "core/tgCast.h"
#include "core/tgTags.h"

#include <iostream>
#include <fstream>
#include <string>

/**
 * Construct our data logger
 */
tgDataLoggerLinearStringBasic::tgDataLoggerLinearStringBasic()
{
  numLinearStrings = 0;
}

/** Virtual base classes must have a virtual destructor. */
tgDataLoggerLinearStringBasic::~tgDataLoggerLinearStringBasic()
{ }

/**
 * Set our filename for output.
 */
void tgDataLoggerLinearStringBasic::setFileName(std::string fileName)
{
  m_fileName = fileName;
}

// Check if the tgModel passed in is actually the type of object we
// want to log.
// TODO: make the string "tgLinearString" a const string or something somewhere,
// so we only have to set it once as opposed to multiple places
// for these various casts.
bool tgDataLoggerLinearStringBasic::isThisMyLoggable(const tgModel* obj) const
{
    bool result;
    if(tgCast::cast<tgModel, tgLinearString>(obj) == 0)
    {
        result = false;
    }
    else
    {
	result =  true;
    }
    return result;
}


void tgDataLoggerLinearStringBasic::writeHeader(tgModel* obj)
{
  // TODO: check if m_fileName has been set, throw exception if not
  // First, check if we're on the right object
  if( tgDataLoggerLinearStringBasic::isThisMyLoggable(obj) )
  {
      // Then, generate the header string for this individual linearString,
      // and write to file.
      // TODO: add tags here.
      std::ofstream tgOutput;
      tgOutput.open(m_fileName.c_str(), std::ios::app);
      std::stringstream linStrName;
      // Add the string number (count) to this string's name
      linStrName << "linStr" << numLinearStrings;
      // Header here is Rest Length, Actual Length, Tension
      tgOutput << linStrName.str() << "_RestLen" << ","
	     << linStrName.str() << "_ActualLen" << ","
	     << linStrName.str() << "_Tension" << ",";
      numLinearStrings++;
      tgOutput.close();
  }
}

void tgDataLoggerLinearStringBasic::render(const tgModel& model) const
{
  // TODO: check if m_fileName has been set, throw exception if not
  // First, check if we're on the right object
  if( tgDataLoggerLinearStringBasic::isThisMyLoggable(&model) )
  {
      // Then, open our output stream and render!
      std::ofstream tgOutput;
      tgOutput.open(m_fileName.c_str(), std::ios::app);
    
      // note that we need to cast here in order to access the actual methods
      // within tgLinearString.
      const tgLinearString* castedLinStr = 
	tgCast::cast<tgModel, tgLinearString>(&model);

      tgOutput << castedLinStr->getRestLength() << ","    
	       << castedLinStr->getCurrentLength() << ","
	       << castedLinStr->getTension() << ",";

      tgOutput.close();
  }
}

