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

#ifndef VERT_CONSTRAINT_FOR_MODEL_H
#define VERT_CONSTRAINT_FOR_MODEL_H

/**
 * @file VertConstraintForModel.h
 * @brief Contains the definition of class VertConstraintForModel.
 * @author Drew Sabelhaus
 * $Id$
 */

// The NTRT core library
#include "core/tgObserver.h"
#include "core/tgSubject.h"
#include "core/tgTags.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ standard library
#include <string>
#include <vector>
#include <map>

// Forward declarations
class TensegrityModel;

/**
 * This "controller" only applies a vertical constraint in onSetup to the bodies passed in.
 */
class VertConstraintForModel : public tgObserver<TensegrityModel>, public tgSubject<VertConstraintForModel>
{
public:
	
  /**
   * Construct a VertConstraintForModel.
   * @param[in] tags, a vector of tags to constrain in the model. Should be descendants of the model.
   */
  VertConstraintForModel(std::vector<std::string> tags);

  // Alternative constructor: take in one tag only if that's easier.
  VertConstraintForModel(std::string tag);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~VertConstraintForModel() { }

  /**
   * Add the constraint
   * @param[in] subject - the TensegrityModel that has bodies to be set to vertical.
   */
  virtual void onSetup(TensegrityModel& subject);
    
  /**
   * Do nothing.
   */
  virtual void onStep(TensegrityModel& subject, double dt);
    
private:

  /**
   * The helper that adds a constraint for one of the tags.
   */
  void addVerticalConstraint(TensegrityModel& subject, std::string tag);

  // private list of the tags.
  std::vector<std::string> m_tags;

};

#endif // VERT_CONSTRAINT_FOR_MODEL_H
