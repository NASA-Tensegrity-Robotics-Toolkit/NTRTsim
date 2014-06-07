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

#ifndef __CONFIGURATION_CLASS_H__
#define __CONFIGURATION_CLASS_H__

/**
 * @file configuration.h
 * @brief A class to read a learning configuration from a .ini file.
 * @date April 2013
 * @author Atil Iscen
 * $Id$
 */

#include <map>
#include <string>

class configuration
  {

  /**---------------------------------------------------------------------------
  // The configuration::data is a simple map string (key, value) pairs.
  // The file is stored as a simple listing of those pairs, one per line.
  // The key is separated from the value by an equal sign '='.
  // Commentary begins with the first non-space character on the line a hash or
  // semi-colon ('#' or ';').
  //
  // Example:
  //   # This is an example
  //   source.directory = C:\Documents and Settings\Jennifer\My Documents\
  //   file.types = *.jpg;*.gif;*.png;*.pix;*.tif;*.bmp
  //
  // Notice that the configuration file format does not permit values to span
  // more than one line, commentary at the end of a line, or [section]s.
  */

public:
   configuration();
   ~configuration();

   std::map <std::string, std::string> data;
    // Here is a little convenience method...
    bool iskey( const std::string& s ) const
      {
      return data.count( s ) != 0;
      }
    // Gets an integer value from a key. If the key does not exist, or if the value
    // is not an integer, throws an int exception.
    //
    int getintvalue( const std::string& key );
    double getDoubleValue(const std::string& key );
	std::string getStringValue(const std::string& key );
    void readFile(const std::string filename);
    void writeToFile(const std::string filename);
};

#endif
