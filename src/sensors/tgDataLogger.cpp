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
 * @file tgDataLogger.cpp
 * @brief Contains the definition of interface class tgDataLogger.
 * @author Brian Tietz, Drew Sabelhaus
 * $Id$
 */

// This applicaiton
#include "tgDataLogger.h"
// Classes that will be rendered
#include "util/tgBaseCPGNode.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgCompressionSpringActuator.h"
#include "core/tgRod.h"
#include "core/abstractMarker.h"
#include "sensors/forceplate/ForcePlateModel.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ standard library
#include <iostream>
#include <fstream>

tgDataLogger::tgDataLogger(std::string fileName) :
m_fileName(fileName)
{}

/** Virtual base classes must have a virtual destructor. */
tgDataLogger::~tgDataLogger()

{ }

void tgDataLogger::render(const tgRod& rod) const
{
    std::ofstream tgOutput;
    tgOutput.open(m_fileName.c_str(), std::ios::app);
    
    btVector3 com = rod.centerOfMass();
    
    tgOutput << com[0] << ","
    << com[1] << ","
    << com[2] << ","
    << rod.mass() << ",";
    
    tgOutput.close();
}
    
void tgDataLogger::render(const tgSpringCableActuator& mSCA) const
{
    std::ofstream tgOutput;
    tgOutput.open(m_fileName.c_str(), std::ios::app);
    
    tgOutput << mSCA.getRestLength() << ","    
    << mSCA.getCurrentLength() << ","
    << mSCA.getTension() << ",";
    
    tgOutput.close();
}

/**
 * Render a tgCompressionSpringActuator
 * As of 2016-08-02, do nothing (not ready to test this out yet, just need
 * to compile.)
 */
void tgDataLogger::render(const tgCompressionSpringActuator& compressionSpringActuator) const
{
    // do nothing.
  //DEBUGGING
  //std::cout << "Rendering a CompressionSpringActuator inside tgDataLogger" << std::endl;
}

/**
 * Render a ForcePlateModel. Output its forces to the log file.
 */
void tgDataLogger::render(const ForcePlateModel& forcePlate) const
{
  //DEBUGGING
  //std::cout << "Rendering a ForcePlateModel inside tgDataLogger" << std::endl;
  // Open up the log
  std::ofstream tgOutput;
  tgOutput.open(m_fileName.c_str(), std::ios::app);
  // Output the forces in X, Y, and Z
  tgOutput << forcePlate.getFx() << ","
  	   << forcePlate.getFy() << ","
  	   << forcePlate.getFz() << ",";
  // Close the output
  // Note that a newline will be added later.
  tgOutput.close();
}

void tgDataLogger::render(const tgModel& model) const
{
  // PLEASE EXCUSE THIS HACK. NOT SURE WHY THE RENDER FUNCTION FOR
  // FORCEPLATEMODEL IS NOT BEING CALLED DIRECTLY.
  // @TODO FIX THIS!!!!!
  if( 1 ) {
    // Detect if the model being passed in is a ForcePlateModel.
    const ForcePlateModel* forcePlate = tgCast::cast<tgModel, ForcePlateModel>(&model);
    if( forcePlate != 0) {
      // Forcibly call the render function for ForcePlateModel inside this class.
      // @TODO figure out why this has to happen.
      render(*forcePlate);
    }
  }
  const std::vector<abstractMarker>& markers = model.getMarkers();
    
  const std::size_t n = 0;
  for (std::size_t i = 0; i < n; i++) {
    std::ofstream tgOutput;
    tgOutput.open(m_fileName.c_str(), std::ios::app);
    
    btVector3 worldPos = markers[i].getWorldPosition();
    
    tgOutput << worldPos[0] << ","    
	     << worldPos[1]  << ","
	     << worldPos[2]  << ",";
            
    tgOutput.close();
    }
}
