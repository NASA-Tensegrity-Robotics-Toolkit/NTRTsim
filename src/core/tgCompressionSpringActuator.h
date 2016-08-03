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

#ifndef SRC_CORE_TG_COMPRESSION_SPRING_ACTUATOR_H
#define SRC_CORE_TG_COMPRESSION_SPRING_ACTUATOR_H

/**
 * @file tgCompressionSpringActuator.h
 * @brief Contains the definition of class tgCompressionSpringActuator.
 * This class assumes a linear spring, F = k \delta x - bV, where the spring begins to
 * apply a force when the length between anchors goes down past restLength.
 * Uses a tgBulletCompressionSpring to calculate and apply forces.
 * @author Drew Sabelhaus, Brian Mirletz
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// This application
#include "tgModel.h"
#include "tgControllable.h"
#include "tgSubject.h"

// for history logging
#include <deque>

// Forward declarations
class tgBulletCompressionSpring;
class tgModelVisitor;
class tgWorld;

/**
 * This class is what should be used to create a compression spring in NTRT.
 * As of 2016-08-02, it does not actuate, though, but is named 'actuator'
 * in the hopes that this functionality will be added.
 */

// This class needs to be a child model of a tgModel.
class tgCompressionSpringActuator : public tgModel, public tgControllable,
                                      public tgSubject<tgCompressionSpringActuator>
{
public: 

  /**
   * The config struct.
   * As of 2016-08-02, since this compression spring is unactuated, the
   * config struct only needs a small number of the parameters in tgSpringCableActuator.
   */
  struct Config
  {
  public:
    
    /**
     * Parameter defaults 
     */
    Config(
	 double s = 1000.0,
	 double d = 10.0,
	 double rL = 0.0,
	 double rot = 0);

    /**
     * List of the parameters themselves.
     * All are passed in to tgBulletCompressionSpring.
     */
    
    /**
     * Linear Hookean stiffness of the spring (k). Must be non-negative.
     * Upper limit depends on the timestep - stiffer springs will
     * require a lower timestep.
     * Units are mass / seconds^2
     */
    double stiffness;
    
    /**
     * Specifies the damping (b) term in the linear force equation.
     * Units are mass / seconds
     * Must be non-negative.
     */
    double damping;

    /**
     * Rest length of the spring. At distances above this amount,
     * no force will be applied.
     * Must be non-negative.
     */
    double restLength;
    
    /**
     * Specifies the rotation around the face of the object its attached
     * to.  +/- PI is the most meaningful. absolute value must be < 2 * PI
     * Units are radians.
     * @todo Is this meaningful for non-rod shapes? Copied from tgSpringCableActuator...
     */
    double rotation;
  };
  
  /**
   * THE MAIN CONSTRUCTOR FOR THIS CLASS.
   * Constructor using tags. Typically called in tgCompressionSpringActuatorInfo.cpp 
   * @param[in] compressionSpring, the tgBulletCompressionSpring object that this controls and logs.
   * Set up in tgCompressionSpringActuatorInfo.cpp
   * @param[in] tags as passed through tgStructure and tgStructureInfo
   * @param[in] config Holds member variables defined here.
   */    
  tgCompressionSpringActuator(tgBulletCompressionSpring* compressionSpring,
         const tgTags& tags,
         tgCompressionSpringActuator::Config& config);
    
  /**
   * Destructor deletes the tgBulletCompressionSpring
   */
  virtual ~tgCompressionSpringActuator();
    
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
   * Functions for interfacing with tgBulletCompressionSpring
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

  /**
   * Returns a pointer to this spring's tgBulletCompressionSpring.
   * Used by tgBulletRenderer.
   */
  virtual const tgBulletCompressionSpring* getCompressionSpring() const
  {
    return m_compressionSpring;
  }

  /**
   * Functions required by tgControllable.
   * As of 2016-08-02, all of these do nothing.
   */

  /**
   * Does nothing as of 2016-08-02.
   * See tgBasicActuator for an implementation that could be re-used here.
   */
  virtual void setControlInput(double input);
	
  /**
   * Does nothing as of 2016-08-02.
   * See tgBasicActuator for an implementation that could be re-used here.
   */
  virtual void setControlInput(double input, double dt);

protected:

    /**
     * the tgBulletCompressionSpring that belongs to this actuator.
     */
    tgBulletCompressionSpring* m_compressionSpring;

    /**
     * A copy of the configuration POD supplied at constuction.
     * This is not const.
     */
    Config m_config;

    // things that were not copied in from tgSpringCableActuator.h:
    //m_restLength
    //m_startLength
    //m_prevVelocity

private:

    /**
     * Helper function to perform what is in common to all constructor bodies.
     */
    void constructorAux();

    /** Integrity predicate. */
    bool invariant() const;

    /**
     * The velocity at the previous timestep, used when calculating the force
     * applied by damping.
     * AS OF 2016-08-02, THIS IS NOT USED SINCE THE DAMPING CAN BE CALCUALTED
     * WITHIN tgBulletCompressionSpring ITSELF.
     */
    double m_prevVelocity;
    
};


#endif
