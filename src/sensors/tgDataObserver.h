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

#ifndef TG_DATA_OBSERVER_H
#define TG_DATA_OBSERVER_H

/**
 * @file tgDataObserver.h
 * @brief Definition of tgObserver class
 * @author Brian Tietz
 * @date April 28, 2014
 * $Id$
 */

#include <fstream>
#include <string>

class tgModel;
class tgDataLogger;

/**
 * A class that dispatches data loggers. Should be included by observers,
 * since they will know when to step this, and we don't have any model
 * specific information here.
 */

class tgDataObserver
{
public:
    tgDataObserver(std::string filePrefix);
    
    /** A class with virtual member functions must have a virtual destructor. */
    virtual ~tgDataObserver();
    
    /**@todo move functions to constructor when possible */
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
    
    ///@todo find a way to move things to the constructor and remove this
    std::string m_filePrefix;
    
    double m_totalTime;
    
    tgDataLogger * m_dataLogger;
};
   
#endif
