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

#ifndef TG_DATA_OBSERVER_BASIC_H
#define TG_DATA_OBSERVER_BASIC_H

/**
 * @file tgDataObserverBasic.h
 * @brief Definition of tgDataObserverBasic class
 * @author Drew Sabelhaus and Brian Tietz Mirletz
 * @date July 27, 2014
 * $Id$
 */

#include <fstream>
#include <string>
#include <vector>
#include "core/tgObserver.h"
#include "tgDataLogger_tgDLR.h"

// Forward declarations of classes
class tgModel;

/**
 * tgDataObserverBasic is a type of tgDataLogger, which observes a model.
 * When notified, this observer calls individual data loggers for different
 * Loggable type objects inside the model.
 * This basic data observer only logs the center-of-mass of rods.
 * Note that there should never be model-specific information in any of these
 * data observer or logger classes.
 * Also note that unlike the original tgDataObserver class, this one is
 * actually a real tgObserver!
 */

class tgDataObserver_tgDLR : public tgObserver<tgModel>
{
public:

    /**
     * Construct a tgDataObserver_tgDLR.
     * @param filePrefix, the base path to the file we save data into.
     * This is just the prefix: the actual saved file has the timestamp
     * of the start of the simulation appended to this prefix.
     * @param loggers a vector of tgDataLogger_tgDLR-s that this observer
     * will match to invidiual tgModel objects that we observe.
     */
    tgDataObserver_tgDLR(std::string filePrefix, 
			std::vector<tgDataLogger_tgDLR*> loggers);
    
    /** A class with virtual member functions must have a virtual destructor. */
    virtual ~tgDataObserver_tgDLR();

    /**
     * Once m_fileName has been set, pass that name to all our individual loggers
     */
    virtual void assignFileNameToLoggers();
    
    /**
     * Create the log file and header during setup of the simulation.
     * @param[in] model, the tgModel we're observing.
     */
    virtual void onSetup(tgModel& model);
    
    /**
     * Dispatch the visitors to the given model and log data
     * @param[in] the number of seconds since the previous call; must be
     * positive
     */
    virtual void onStep(tgModel& model, double dt);
    
    /** @todo add reset method so we can start a new file when
     * the simulation resets */

private:

    std::ofstream tgOutput;
    
    std::string m_fileName;

    // How often do we want to sample?
    // This parameter indicates how many timesteps to take before recording
    // a sample: for example 10 timesteps * 0.0001 seconds = logs every 0.001 sec.
    int logging_period;

    // counting how many timesteps, to compare to frequency
    int count_logging_timesteps;
    
    // keeping track of time for the timestamp output
    double m_totalTime;
    double m_timeBetweenLogWrites;
    
    // References to all the individual loggers we'll call to actually
    // output data.
    std::vector<tgDataLogger_tgDLR*> m_loggers;
    
};
   
#endif
