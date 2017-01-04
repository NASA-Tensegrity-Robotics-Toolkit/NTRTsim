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
 * @file tgSenseable.h
 * @brief Constains definition of mixin class tgSenseable, for objects that can be sensed.
 * @author Drew Sabelhaus
 * $Id$
 */

#ifndef TG_SENSEABLE_H
#define TG_SENSEABLE_H

// From the C++ standard library:
#include <iostream> //for strings
#include <assert.h> //for assertions

//#include <sstream>
//#include <vector>
//#include <set>

//#include <cstdio>
//#include <stdlib.h> //atoi
//#include <algorithm>

/**
 * This class defines methods for use with sensors.
 * If an object (really, mostly just tgModel and its descendants) can be sensed,
 * inherit from this class and re-define ALL the methods.
 * Note that we make everything pure virtual here to force re-definition.
 * Sensing data from objects occurs in two places in the NTRTsim workflow.
 * First, when setting up the simulation, a heading for the data is given.
 * This describes the data that will be returned, in a comma-separated-value form.
 * Second, when the simulation is running, the data itself can be taken.
 * Note that it's up to the caller (e.g., a tgDataLogger2) to match up the headings
 * with the data.
 */
class tgSenseable
{
public:

  // Constructor does nothing, since this class
  // contains no data.
  tgSenseable() {}

  // Similarly, destructor does nothing.
  ~tgSenseable() {}

  /**
   * Create a descriptive heading for all the data that this class can return.
   * This will be a CSV string, with the number of columns the same as the
   * number of columns output by the getData function below.
   */
  virtual std::string getDataHeading() = 0;

  /**
   * Return the data from this class itself.
   * Note that this MUST be the same number of CSV columns as is returned by
   * the getDataHeading function.
   */
  virtual std::string getData() = 0;

  // TO-DO: should any of this be const?

};

#endif //TG_SENSEABLE_H
