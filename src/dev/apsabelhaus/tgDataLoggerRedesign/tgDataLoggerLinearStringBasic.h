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

#ifndef TG_DATA_LOGGER_LINEARSTRING_BASIC_H
#define TG_DATA_LOGGER_LINEARSTRING_BASIC_H

/**
 * @file tgDataLoggerLinearStringBasic.h
 * @brief Contains the definition of class tgDataLoggerLinearStringBasic
 * @author Drew Sabelhaus and Brian Tietz
 * $Id$
 */
 
#include "tgDataLogger_tgDLR.h"

#include <string>

// Forward declarations
class tgModel;
class tgSpringCableActuator;

/**
 * This actual data logger is a tgModelVisitor, which has the methods
 * 'render' that are called upon visit.
 */
class tgDataLoggerLinearStringBasic : public tgDataLogger_tgDLR {
    
public:

    /** 
     * Construct a tgDataLoggerLinearStringBasic
     */
    tgDataLoggerLinearStringBasic();
    
    /** Virtual base classes must have a virtual destructor. */
    virtual ~tgDataLoggerLinearStringBasic();
  
    /**
     * Render a tgModel
     * @param[in] model a const reference to a tgModel to log data. Note
     * that this class only actually operates on tgRod(s), but we check 
     * this inside the method.
     */
    virtual void render(const tgModel& model) const;

    /**
     * Set our filename for output.
     */
    virtual void setFileName(std::string fileName);

    /**
     * Check if the tgModel passed in is actually the type of model we're
     * listening for.
     * @param[in] obj a const reference to a tgModel that may or may not
     * actually be a tgRod.
     */
    virtual bool isThisMyLoggable(const tgModel* obj) const;

    /**
     * Return the file header for a tgRod.
     * @param[in] obj a reference to a tgModel that may or may not actually be
     * a tgRod. We do nothing if it's not a tgRod.
     */
    virtual void writeHeader(tgModel* obj);

private:
    
    std::string m_fileName;

    // a counter for creating the header file
    int numLinearStrings;

};

#endif
