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

#ifndef TG_DATA_LOGGER_TGDLR_H
#define TG_DATA_LOGGER_TGDLR_H

/**
 * @file tgDataLogger_tgDLR.h
 * @brief Contains the definition of interface class tgDataLogger_tgDLR
 * @author Drew Sabelhaus
 * $Id$
 */

// Forward declarations
//class tgLinearString;
class tgModel;
//class tgRod;

// This library
#include "tgModelVisitor_tgDLR.h"
#include <string>

/**
 * Interface for data observers
 */
class tgDataLogger_tgDLR : public tgModelVisitor_tgDLR
{
    
public:

  /* Constructor */
  tgDataLogger_tgDLR() {}

  /** Virtual base classes must have a virtual destructor. */
  virtual ~tgDataLogger_tgDLR() { }

  /**
   * Set our filename for output.
   */
  virtual void setFileName(std::string fileName) {}

  /**
   * Check if an object is to be logged by this data
   */
  virtual bool isThisMyLoggable(const tgModel* obj) const {}

  /**
   * Write the header for an object that this class wants to log.
   * This is like render() for the superclass of tgModelVisitor_tgDLR,
   * but specific to data loggers.
   */
  virtual void writeHeader(tgModel* obj) {}

};

#endif
