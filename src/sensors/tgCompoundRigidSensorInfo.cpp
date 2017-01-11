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
 * @file tgCompoundRigidSensorInfo.cpp
 * @brief Contains the implementation of concrete class tgCompoundRigidSensorInfo
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "tgCompoundRigidSensorInfo.h"
// Other includes from NTRTsim
//#include "tgCompoundRigidSensor.h"
#include "core/tgModel.h"
#include "core/tgSenseable.h"
#include "core/tgCast.h"
// Other includes from the C++ standard library
#include <stdexcept>
#include <map> // for maps, used in checking number of rigid compounds
#include <sstream> // for ease of converting tags to strings.
#include <iostream> // for writing output to the terminal
// Includes from boost
#include <boost/regex.hpp>

/**
 * Nothing to do in this constructor. A sensor info doesn't have any data.
 */
tgCompoundRigidSensorInfo::tgCompoundRigidSensorInfo()
{
}

/**
 * Similarly, empty destructor.
 */
tgCompoundRigidSensorInfo::~tgCompoundRigidSensorInfo()
{
}

/**
 * This function returns true if pSenseable is a tgModel with:
 * (1) at least two descendants
 * (2) at least two of the descendants, both of which can be cast to tgBaseRigids,
 *     have matching "compound_XXXXXX" tags
 */
bool tgCompoundRigidSensorInfo::isThisMySenseable(tgSenseable* pSenseable)
{
  // Check for null pointer.
  if ( pSenseable == NULL) {
    throw std::invalid_argument("pSenseable was NULL inside tgCompoundRigidSensorInfo.");
  }
  else {
    // Condition 1: cast to tgModel, since compound rigids are always tgModels,
    // and since we need the tags from the objects and tgSenseables don't have tags.
    tgModel* pModel = tgCast::cast<tgSenseable, tgModel>(pSenseable);
    if (pModel == 0 ) {
      return 0;
    }
    // Then, get the descendants, and check if there are at least two.
    // Note that we're using tgModel's getDescendants function here instead
    // of tgSenseable's getSenseableDescendants, since we want the pointers to
    // be tgModel pointers.
    std::vector<tgModel*> descendants = pModel->getDescendants();
    // if size is less than two, return false.
    if (descendants.size() < 2) {
      return 0;
    }
    // Condition 2: this one is more involved. Need to iterate through the list,
    // tracking the number of descendants that are (a) rigid bodies and (b) have
    // a specific compound tag.
    
    // This map will store the number of valid rigids with a given compound tag.
    std::map<std::string, int> compounds;
    // Create the regular expression that will be used to find compound tags
    // The following regex should match the characters "compound_" with
    // six alphanumeric characters at the end. (This is the hash that
    // tgRigidAutoCompounder creates.)
    boost::regex compound_regex("compound_\\w{6}");
    
    // Iterate through the descendants
    //DEBUGGING
    // std::cout << "Descendants has size: " << descendants.size() << std::endl;
    for (size_t i=0; i < descendants.size(); i++){
      // If this descendant has a compound tag...
      // (a) get the tags in string form
      std::stringstream tagstream;
      tagstream << descendants[i]->getTags();
      std::string tags = tagstream.str();
      //std::cout << "Tags are: " << tags << std::endl;
      // (b) use a regular expression to pick out any compound tags
      boost::smatch matches;
      bool anymatches = boost::regex_search(tags, matches, compound_regex);
      // If there are zero matches, do nothing: this descendant is not compound.
      // Note that matches is only initialized if anymatches is true.
      // In other words, we CANNOT do a check of matches.size() here instead,
      // since if matches.size() == 0, then matches will not have been initialized.
      if( anymatches ) {
	// Note that there should be AT MOST one match.
	// No rigid body should ever be part of more than one compound: if
	// it was part of two compounds, those compounds would be the same!!
	// TO-DO: write some verifying code (maybe in tgRigidAutoCompounder)
	// to enforce this.
	if (matches.size() >= 2 ){
	  throw std::runtime_error("A tgModel has more than one compound tag in its tag list, inside tgCompoundRigidSensorInfo. This is impossible, and something is very wrong.");
	}
	//std::cout << "Found a compound! Original tags were: "
	//	  << tags << " and regex match is: " << matches.str(0) << std::endl;

	// Now, add to the index of counts for this tag.
	compounds[matches.str(0)] += 1;
      }
    }
    // Finally, count the number of descendants for each compound.
    // If there are at least two, return true.
    // Iterate over the map:
    std::map<std::string, int>::iterator it;
    for( it = compounds.begin(); it != compounds.end(); ++it){
      // The 'second' field of this iterator is the value.
      // We need at least two rigids in order to have a proper compound.
      if( (it->second) >= 2 ) {
	return 1;
      }
      else if( (it->second) == 1) {
	// It should never happen that there is only one object with a
	// compound tag...
	throw std::runtime_error("There is only one object with a compound tag. That's not possible, compound tgModels should be at least two objects. Exiting.");
      }
    }
    // If there were no compound tags,
    return 0;
  }
}

/**
 * Create a tgCompoundRigidSensor for a tgModel.
 */
tgSensor* tgCompoundRigidSensorInfo::createSensor(tgSenseable* pSenseable)
{
  //CHECK: the caller SHOULD HAVE made sure that pSenseable
  // was a tgRod pointer. If not, complain!!
  if (!isThisMySenseable(pSenseable)) {
    throw std::invalid_argument("pSenseable is NOT a valid tgModel or does not have compound rigid bodies, inside tgCompoundRigidSensorInfo.");
  }
  // Then, if the program hasn't quit, make the sensor.
  // Note that we cast the pointer here, knowing that it will succeed.
  //return new tgCompoundRigidSensor( tgCast::cast<tgSenseable, tgRod>(pSenseable) );
  // DEBUGGING: make this compile so the above code can be run.
  std::cout << "Returning a null pointer for tgCompRigSens.createSensor." << std::endl;
  return NULL;
}
