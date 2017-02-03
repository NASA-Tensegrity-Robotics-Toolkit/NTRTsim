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
#include "tgCompoundRigidSensor.h"
#include "core/tgModel.h"
#include "core/tgBaseRigid.h" // for checking descendants via casting.
#include "core/tgSenseable.h"
#include "core/tgCast.h"
// Other includes from the C++ standard library
#include <stdexcept>
#include <sstream> // for ease of converting tags to strings.
#include <iostream> // for writing output to the terminal
#include <algorithm> // // for std::find, used with blacklist.
// Includes from boost
#include <boost/regex.hpp>

/**
 * The regular expression for use in picking out compound tags.
 * Define it here, since it will be used in a variety of places.
 * The following regex should match the characters "compound_" with
 * six alphanumeric characters at the end. (This is the hash that
 * tgRigidAutoCompounder creates.)
 */
static const boost::regex compound_regex("compound_\\w{6}");

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
 * This function returns a map of tags and the count of tgModel descendants with
 * those tags.
 */
std::map<std::string, int> tgCompoundRigidSensorInfo::getCompoundTags(tgModel* pModel)
{
  // Check for null pointer.
  if ( pModel == NULL) {
    throw std::invalid_argument("pModel was NULL inside tgCompoundRigidSensorInfo.");
  }
  // Then, get the descendants.
  // Note that we're using tgModel's getDescendants function here.
  std::vector<tgModel*> descendants = pModel->getDescendants();
  // This map will store the number of valid rigids with a given compound tag.
  std::map<std::string, int> compounds;
  
  // Iterate through the descendants, search for tags,
  // and count them up.
  for (size_t i=0; i < descendants.size(); i++){
    // Check if this descendant has a compound tag...
    // (a) get the tags in string form
    std::stringstream tagstream;
    tagstream << descendants[i]->getTags();
    std::string tags = tagstream.str();
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
      // Confirm that the specific descendant is actually a rigid body.
      // It's possible that some not-rigid-bodies could have erroneous tags...
      tgBaseRigid* pBaseRigid =
	tgCast::cast<tgSenseable, tgBaseRigid>(descendants[i]);
      if (pBaseRigid == 0 ) {
	throw std::runtime_error("A tgModel that is NOT a rigid body has a compound tag attached to it. Only rigid bodies should have compound tags.");
      }
      // Finally, if everything is good, add to the index of counts for this tag.
      compounds[matches.str(0)] += 1;
    }
  }
  
  // Return the completed list. If descendants was size zero,
  // then this map will not have any entries.
  return compounds;
}

// Quick check: is this tag in the blacklist?
bool tgCompoundRigidSensorInfo::isBlacklisted(std::string tag)
{
  // Use the std::find algorithm, with an iterator.
  std::vector<std::string>::iterator it;
  it = std::find( blacklist.begin(), blacklist.end(), tag);
  if( it == blacklist.end() ) {
    // if the iterator has looked through the whole list
    // and didn't find anything, the tag is not in the blacklist.
    return 0;
  }
  else {
    return 1;
  }
}

/**
 * This function returns true if pSenseable is a tgModel with:
 * (1) at least two descendants
 * (2) at least two of the descendants, both of which can be cast to tgBaseRigids,
 *     have matching "compound_XXXXXX" tags
 * (3) at least one of the tags does NOT exist in the blacklist.
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
    // Then, get the compound tags for this model.
    std::map<std::string, int> compounds = getCompoundTags(pModel);
    
    // note that we don't need to expressly check if there are
    // two or more descendants of this model. If there are less, then
    // 'compounds' will just have no entries.

    // Condition 2:  
    // Are there any valid compounds?
    // Iterate over the map:
    std::map<std::string, int>::iterator it;
    for( it = compounds.begin(); it != compounds.end(); ++it){
      // The 'second' field of this iterator is the value.
      // We need at least two rigids in order to have a proper compound.
      if( (it->second) >= 2 ) {
	// Condition 3: finally, check and return true iff this compound
	// tag is not in the blacklist.
	if( !isBlacklisted( it->first ) ){
	  return 1;
	}
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
std::vector<tgSensor*> tgCompoundRigidSensorInfo::createSensorsIfAppropriate(tgSenseable* pSenseable)
{
  //CHECK: the caller SHOULD HAVE made sure that pSenseable
  // satisfied the conditions of sensor creation.
  if (!isThisMySenseable(pSenseable)) {
    throw std::invalid_argument("pSenseable is NOT a valid tgModel or does not have non-blacklisted compound rigid bodies, inside tgCompoundRigidSensorInfo.");
  }
  // Then, if the program hasn't quit, make the sensor.
  std::vector<tgSensor*> newSensors;

  /**
   * (1) Get the compound tags from within this tgModel.
   * (2) Remove any compound tags that are in blacklist.
   * (3) Create sensors for all remaining tags.
   * (4) Blacklist the tags that were just now passed in to the sensor 
   *     constructor.
   */

  // First, need to cast the pointer to tgModel.
  tgModel* pModel = tgCast::cast<tgSenseable, tgModel>(pSenseable);
  if (pModel == 0 ) {
    throw std::invalid_argument("pSenseable is not a tgModel inside tgCompoundRigidSensorInfo::createSensorsIfAppropriate.");
  }

  // (1)
  std::map<std::string, int> compounds = getCompoundTags(pModel);
  // (2)
  for( size_t i=0; i < blacklist.size(); i++) {
    // Note that it's valid to erase a non-existant key from a map.
    compounds.erase(blacklist[i]);
  }
  // (3)
  // Use an iterator over the map
  std::map<std::string, int>::iterator it;
  for( it = compounds.begin(); it != compounds.end(); ++it){
    // Add a new sensor for the specified tag.
    // At this point, it should be GUARANTEED that the tag
    // exists in the model, so this should always be valid...
    // Note that it->first points to a tag in this map.
    newSensors.push_back( new tgCompoundRigidSensor(pModel, it->first) );
    // (4)
    // Now that a sensor for this tag has been created, add
    // this tag to the blacklist.
    blacklist.push_back( it->first );
  }

  // This list should now be populated.
  return newSensors;
}
