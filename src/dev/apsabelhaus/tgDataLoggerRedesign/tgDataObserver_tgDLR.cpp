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
 * @file tgDataObserver_tgDLR.cpp
 * @brief Implementation of tgDataObserver_tgDLR class
 * @author Drew Sabelhaus and Brian Tietz Mirletz
 * @date July 27, 2014
 * $Id$
 */

#include "tgDataObserver_tgDLR.h"

#include "core/tgCast.h"
#include "core/tgModel.h"
#include "core/tgRod.h"
#include "core/tgString.h"

#include <iostream>
#include <sstream>  
#include <time.h>

/**
 * Instantiate a tgDataObserver_tgDLR.
 * Open up our output file, make sure we release all previous data loggers
 * @todo make sure we're free-ing old loggers here...
 */

tgDataObserver_tgDLR::tgDataObserver_tgDLR(std::string filePrefix,
					 std::vector<tgDataLogger_tgDLR*> loggers) :
m_loggers(loggers)
{
    // set up our constants
    logging_period = 10;
    count_logging_timesteps = 0;
    m_totalTime = 0.0;
    m_timeBetweenLogWrites = 0.0;

    /**
     * Create the full filename
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
    m_fileName = filePrefix + fileTime;
    std::cout << m_fileName << std::endl;

    // Pass this full filename to all our loggers
    assignFileNameToLoggers();
}

tgDataObserver_tgDLR::~tgDataObserver_tgDLR()
{ 
    // TODO: check if we need to delete things here??
    //m_loggers.clear();
    tgOutput.close();
}

void tgDataObserver_tgDLR::assignFileNameToLoggers()
{
  // for each of our m_loggers, pass in the full filename,
  // NOT the prefix.
  // TODO: check that m_fileName has been set, throw exception if not.
  for(std::size_t i=0; i < m_loggers.size(); i++)
    {
      m_loggers[i]->setFileName(m_fileName);
    }
}

/**@todo move functions to constructor when possible */
void tgDataObserver_tgDLR::onSetup(tgModel& model)
{
   
    // First time opening this, so nothing to append to: no
    // need for std::ios::app.
    tgOutput.open(m_fileName.c_str());
    
    std::vector<tgModel*> children = model.getDescendants();

    // TODO: Check here if we have a logger in our list that does NOT
    // correspond to a type we'll be logging: i.e.,
    // for each type that a logger will look for, search children
    // until we find at least one object with that type

    // Start the log file header with Time
    tgOutput << "Time" << ",";
    tgOutput.close();

    // Create the headers for the log file. Loop through each child
    // and if it's of a type that we have a logger for, ask that logger
    // to give us a string for the header, then append it to the log file.

    for(std::size_t i=0; i < children.size(); i++) {
        for(std::size_t j=0; j < m_loggers.size(); j++) {

	    // if this logger is looking for this object...
	    if( m_loggers[j]->isThisMyLoggable(children[i]) ) {

	      // ask the logger to write its header
	      m_loggers[j]->writeHeader(children[i]);
	    }
        }
    }

    // Append a newline to the output file.
    tgOutput.open(m_fileName.c_str(), std::ios::app);
    tgOutput << std::endl;
    tgOutput.close();
}


/**
 * Dispatch the visitors to the given model and log data
 * @param[in] the number of seconds since the previous call; must be
 * positive
 */
void tgDataObserver_tgDLR::onStep(tgModel& model, double dt)
{
    // check if we log during this iteration of the simulation
    count_logging_timesteps += 1;
    m_timeBetweenLogWrites += dt;

    if( count_logging_timesteps >= logging_period)
    {
      // Append Timestamp
      m_totalTime += m_timeBetweenLogWrites;
      tgOutput.open(m_fileName.c_str(), std::ios::app);
      tgOutput << m_totalTime << ",";
      tgOutput.close();
    
      // Call each of the loggers to output their data
      // Similar to above, iterate through the descendents of the model
      // and if the logger is appropriate, dispatch it
      std::vector<tgModel*> children = model.getDescendants();

      for(std::size_t i=0; i < children.size(); i++) {
          for(std::size_t j=0; j < m_loggers.size(); j++) {

	      // if this logger is looking for this objecct...
	      if( m_loggers[j]->isThisMyLoggable(children[i]) ) 
	      {
	          // render (output) this subclass of tgModel
		  // note that we need to de-ref the pointer here due to the
		  // function definition of onVisit. But that's OK, it's const.
#if (0)
		  children[i]->onVisit(*m_loggers[j]);
#endif //deprecated 
	      }
	  }
      }
    
      // Append a newline character
      tgOutput.open(m_fileName.c_str(), std::ios::app);
      tgOutput << std::endl;
      tgOutput.close();
      
      // reset the count of steps
      count_logging_timesteps = 0.0;
      m_timeBetweenLogWrites = 0.0;
    }

}
