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
 * @file tscDataObserver.cpp
 * @brief Implementation of tgObserver class
 * @author Brian Tietz
 * @date April 28, 2014
 * $Id$
 */

#include "tscDataObserver.h"

#include "tscDataLogger.h"

#include "core/tgCast.h"
#include "core/tgModel.h"
#include "core/tgRod.h"
#include "core/tgString.h"

#include "core/tgSpringCableActuator.h"

#include "examples/learningSpines/BaseSpineModelLearning.h"

#include <iostream>
#include <sstream>  
#include <time.h>
#include <stdexcept>

tscDataObserver::tscDataObserver(std::string filePrefix) :
m_totalTime(0.0),
m_dataLogger(NULL),
m_filePrefix(filePrefix)
{

}

/** A class with virtual member functions must have a virtual destructor. */
tscDataObserver::~tscDataObserver()
{ 
    delete m_dataLogger;
    tgOutput.close();
}

/**@todo move functions to constructor when possible */
void tscDataObserver::onSetup(BaseSpineModelLearning& model)
{
	/*
	 * Adapted from: http://www.cplusplus.com/reference/clibrary/ctime/localtime/
	 * Also http://www.cplusplus.com/forum/unices/2259/
	*/
    time_t rawtime;
    tm* currentTime;
    int fileTimeSize = 64;
    char fileTime [fileTimeSize];
    
    time (&rawtime);
    currentTime = localtime(&rawtime);
    strftime(fileTime, fileTimeSize, "%m%d%Y_%H%M%S.txt", currentTime);
    m_fileName = m_filePrefix + fileTime;
    std::cout << m_fileName << std::endl;
    
    if (m_dataLogger != NULL)
    {
        // prevent leaks on loop behavior (better than teardown?)
        delete m_dataLogger;
    }
    
    m_dataLogger = new tscDataLogger(m_fileName);
    
    m_totalTime = 0.0;
    
    // First time opening this, so nothing to append to
    tgOutput.open(m_fileName.c_str());
    
	if (!tgOutput.is_open())
	{
		throw std::runtime_error("Logs does not exist. Please create a logs folder in your build directory or update your cmake file");
	}
    
    std::vector<tgModel*> children = model.getDescendants();
    
    /*
     * Numbers to ensure uniqueness of variable names in log
     * May be redundant with tag
     */
    int stringNum = 0;
    int rodNum = 0;
    
    tgOutput << "Time" << ",";
    
    for (std::size_t i = 0; i < children.size(); i++)
    {
        /* If its a type we'll be logging, record its name and the 
         * variable types we'll be logging later
         */
        std::stringstream name;
        
        if(tgCast::cast<tgModel, tgSpringCableActuator>(children[i]) != 0) 
        {
            name << children[i]->getTags() <<  " " << stringNum;
            tgOutput <<  name.str() << "_RL" << ","
            <<  name.str() << "_AL" << ","
            <<  name.str() << "_Ten" << ",";
            stringNum++;
        }
        else if(tgCast::cast<tgModel, tgRod>(children[i]) != 0)
        {

        }
        // Else do nothing since tscDataLogger won't touch it
    }
    
    int numSeg = model.getSegments();
    
    for (int i = 0; i < numSeg; i++)
    {
        std::stringstream name;
        
        name << "segment_" << i+1;
        tgOutput << name.str() << "_X" << ","
        << name.str() << "_Y" << ","
        << name.str() << "_Z" << ",";
    }
    
    tgOutput << std::endl;
    
    tgOutput.close();
}

/**
 * Dispatch the visitors to the given model and log data
 * @param[in] the number of seconds since the previous call; must be
 * positive
 */
void tscDataObserver::onStep(BaseSpineModelLearning& model, double dt)
{
    m_totalTime += dt;
    tgOutput.open(m_fileName.c_str(), std::ios::app);
    tgOutput << m_totalTime << ",";
    tgOutput.close();
    
    model.onVisit(*m_dataLogger);
    
    tgOutput.open(m_fileName.c_str(), std::ios::app);
    
    int numSeg = model.getSegments();
    
    for (int i = 0; i < numSeg; i++)
    {
        
        std::vector<double> com = model.getSegmentCOM(i);
        
        tgOutput << com[0] << ","
        << com[1] << ","
        << com[2] << ",";
    }
    
    tgOutput << std::endl;
    tgOutput.close();
}
