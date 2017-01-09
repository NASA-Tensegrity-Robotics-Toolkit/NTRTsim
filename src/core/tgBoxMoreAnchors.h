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

#ifndef TG_BOX_MORE_ANCHORS_H
#define TG_BOX_MORE_ANCHORS_H

/**
 * @file tgBoxMoreAnchors.h
 * @brief Create a box shape. This class supports more than two anchor points.
 * @author Drew Sabelhaus
 * $Id$
 */

// This application
#include "tgBox.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <vector>

/**
 * This class is a child of tgBox, with nothing changed.
 * The purpose of this class is to force the use of tgBoxMoreAnchorsInfo,
 * since the code could potentially get messy if a regular tgBox was able 
 * to be created with a tgBoxMoreAnchorsInfo.
 * More things may be added if needed?
 * But for now, use it exactly the same as tgBox, just with the different Info.
 */
class tgBoxMoreAnchors : public tgBox
{
public:

  /**
   * Use the config struct from tgBox. Only the one constructor (like tgBox.)
   */
    
  tgBoxMoreAnchors(btRigidBody* pRigidBody,
	const tgTags& tags,
	const double length);
    
  /** A class with a virtual memeber function requires a virtual destructor. */
  virtual ~tgBoxMoreAnchors();

  /**
   * Almost all the other methods can be inherited directly from tgBox, no need
   * to re-declare or re-define them.
   */

  // We do need to be able to have a rendering for this specific type
  // of box, though.
  virtual void onVisit(const tgModelVisitor& v) const;
  
  /**
   * Since m_length is a private variable, it needs to be stored in this class
   * and returned from here.
   * @TODO: this is messy! Two copies of m_length are stored: one in the parent,
   * and one in this class. Even though we'll make them the same, this is BAD,
   * should do something like (for example) make length protected in the parent.
   */
  double length() const { return m_length; }


private:

    /** Integrity predicate. */
    bool invariant() const;

private:
    
    /** The box's length. */
    const double m_length;
};

#endif // TG_BOX_MORE_ANCHORS_H
