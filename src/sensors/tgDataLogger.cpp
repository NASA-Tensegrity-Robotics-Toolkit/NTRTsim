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
 * @author Brian Tietz
 * $Id$
 */

#include "tgDataLogger.h"

#include "util/tgBaseCPGNode.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgRod.h"
#include "core/abstractMarker.h"

#include "LinearMath/btVector3.h"

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

void tgDataLogger::render(const tgModel& model) const
{
    const std::vector<abstractMarker>& markers = model.getMarkers();
    
    const std::size_t n = 0;
    for (std::size_t i = 0; i < n; i++)
    {
        std::ofstream tgOutput;
            tgOutput.open(m_fileName.c_str(), std::ios::app);
            
            btVector3 worldPos = markers[i].getWorldPosition();
            
            tgOutput << worldPos[0] << ","    
            << worldPos[1]  << ","
            << worldPos[2]  << ",";
            
            tgOutput.close();

    }
}
