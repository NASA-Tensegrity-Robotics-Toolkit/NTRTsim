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

#ifndef SRC_CORE_TG_UNIDIRECTIONAL_COMPRESSION_SPRING_ACTUATOR_H
#define SRC_CORE_TG_UNIDIRECTIONAL_COMPRESSION_SPRING_ACTUATOR_H

/**
 * @file tgUnidirectionalCompressionSpringActuator.h
 * @brief Contains the definition of class tgUnidirectionalCompressionSpringActuator.
 * This class assumes a linear spring, F = k \delta x - bV, where the spring begins to
 * apply a force when the length between anchors goes down past restLength.
 * Uses a tgBulletUnidirectionalCompressionSpring to calculate and apply forces.
 * @author Drew Sabelhaus, Brian Mirletz
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// This application
#include "tgModel.h"
#include "tgControllable.h"
#include "tgSubject.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"

// This class inherits from tgCompressionSpringActuator
#include "tgCompressionSpringActuator.h"

// for history logging
#include <deque>

// Forward declarations
class tgBulletUnidirectionalCompressionSpring;
class tgModelVisitor;
class tgWorld;

/**
 * This class is what should be used to create a compression spring in NTRT.
 * As of 2016-08-02, it does not actuate, though, but is named 'actuator'
 * in the hopes that this functionality will be added.
 * Since it uses a tgBulletUnidirectionalCompressionSpring, the force applied
 * is only along one direction, not the full 3D vector of distance between nodes.
 * E.g., if direction is (0, 1, 0), then the applied force is always in the vertical
 * direction, and is proportional to only the vertical distance between nodes.
 */

// This class needs to be a child model of a tgModel, but that's
// taken care of by inheriting from tgCompressionSpringActuator
class tgUnidirectionalCompressionSpringActuator : public tgCompressionSpringActuator
{
public: 

  /**
   * The config struct.
   * As of 2016-08-02, since this compression spring is unactuated, the
   * config struct only needs a small number of the parameters in tgSpringCableActuator.
   */
  struct Config : public tgCompressionSpringActuator::Config
  {
  public:
    
    /**
     * Parameter defaults 
     * @TODO: Is this creating a memory leak by immediately copying the 
     * result of new btVector3 ?
     */
    Config(
	 bool iFEA = false,
	 double s = 1000.0,
	 double d = 10.0,
	 double rL = 0.0,
	 bool moveCPA = true,
	 bool moveCPB = true,
	 btVector3 * dir = (new btVector3(0, 1, 0)) );

    /**
     * List of the new parameters.
     * See tgBulletCompressionSpringActuator for info about the others.
     * All are passed in to tgBulletUnidirectionalCompressionSpring.
     */

    /**
     * Direction vector along which the force from this spring acts.
     * Should be checked in constructor: only allowed to be a unit vector
     * along one of the three axes.
     */
    btVector3 * direction;
  };
  
  /**
   * THE MAIN CONSTRUCTOR FOR THIS CLASS.
   * Constructor using tags. Typically called in 
   * tgUnidirectionalCompressionSpringActuatorInfo.cpp .
   * @param[in] compressionSpring, the tgBulletUnidirectionalCompressionSpring 
   * object that this controls and logs.
   * Set up in tgUnidirectionalCompressionSpringActuatorInfo.cpp
   * @param[in] tags as passed through tgStructure and tgStructureInfo
   * @param[in] config Holds member variables defined here.
   */    
  tgUnidirectionalCompressionSpringActuator(
	 tgBulletUnidirectionalCompressionSpring* compressionSpring,
         const tgTags& tags,
         tgUnidirectionalCompressionSpringActuator::Config& config);
    
  /**
   * Destructor does nothing.
   */
  virtual ~tgUnidirectionalCompressionSpringActuator();
    
  /**
   * Notifies observers of setup, calls setup on children
   * @param[in] world, the tgWorld the models are being built into
   */
  virtual void setup(tgWorld& world);
    
  /**
   * Notifies observers of teardown, teardown any children
   */
  virtual void teardown();
    
  /**
   * Step dt forward with the simulation.
   * Notifies observers of step, applies forces to rigid bodies via
   * tgBulletCompressionSpring, logs history if desired, steps children.
   * @param[in] dt, must be >= 0.0
   */    
  virtual void step(double dt);
    
  /**
   * Double dispatch function for a tgModelVisitor. This object
   * will pass itself back to the visitor. Used for rendering and 
   * data logging as of May 2014.
   * @param[in] r, the visiting tgModelVisitor
   */
  virtual void onVisit(const tgModelVisitor& r) const;
    
  /**
   * Functions for interfacing with tgBulletUnidirectionalCompressionSpring
   */

  /**
   * Returns the starting distance between the two anchors.
   * Note that this has little physical meaning, since the spring
   * may not be applying a force at that distance.
   */
  //virtual const double getStartDistance() const;

  /**
   * Returns the current distance between the two anchors.
   * This isn't "length", really, since the spring length really
   * is either:
   *     restLength (if getCurrentDistance is greater than restLength)
   *     getCurrentDistance (if getCurrentDistance is less than restLength).
   */
  //virtual const double getCurrentDistance() const;

  /**
   * Returns the rest length of the spring, e.g., the Config restLength parameter.
   */
  //virtual const double getRestLength() const;

  /**
   * Returns the current force in the spring.
   * This is equivalent to "tension" in a sense, but since this is a 
   * compression spring, it's better termed "spring force."
   */
  //virtual const double getSpringForce() const;


protected:

    /**
     * A copy of the configuration POD supplied at constuction.
     * This is not const.
     * Override the base config to get the extra parameters.
     */
    tgUnidirectionalCompressionSpringActuator::Config m_config;

private:

    /**
     * Since the superclass, tgCompressionSpringActuator, holds on to the
     * compression spring, we may need to cast it when calling certain methods
     * (if there are methods that tgUnidirectionalCompressionSpring has that
     * are not present in tgCompressionSpring). 
     * Here would be a good place to make a function that nicely does that cast.
     */
    
    /**
     * Helper function to perform what is in common to all constructor bodies.
     */
    void constructorAux();

    /** Integrity predicate. */
    bool invariant() const;

};


#endif
