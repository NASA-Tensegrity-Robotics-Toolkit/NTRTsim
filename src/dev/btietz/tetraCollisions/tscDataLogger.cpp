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
 * @file tscDataLogger.cpp
 * @brief Contains the definition of interface class tscDataLogger.
 * @author Brian Tietz
 * $Id$
 */

#include "tscDataLogger.h"

#include "util/tgBaseCPGNode.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgRod.h"

#include <iostream>
#include <fstream>

tscDataLogger::tscDataLogger(std::string fileName) :
m_fileName(fileName)
{}

/** Virtual base classes must have a virtual destructor. */
tscDataLogger::~tscDataLogger()

{ }

void tscDataLogger::render(const tgRod& rod) const
{
    std::ofstream tgOutput;
    tgOutput.open(m_fileName.c_str(), std::ios::app);
    
    btVector3 com = rod.centerOfMass();
#if (0)    
    tgOutput << com[0] << ","
    << com[1] << ","
    << com[2] << ","
    << rod.mass() << ",";
#endif    
    tgOutput.close();
}
    
void tscDataLogger::render(const tgSpringCableActuator& mSCA) const
{
        std::ofstream tgOutput;
    tgOutput.open(m_fileName.c_str(), std::ios::app);
    
    tgOutput << mSCA.getRestLength() << ","    
    << mSCA.getCurrentLength() << ","
    << mSCA.getTension() << ",";
    
    tgOutput.close();
}

void tscDataLogger::render(const tgModel& model) const
{
    
}
