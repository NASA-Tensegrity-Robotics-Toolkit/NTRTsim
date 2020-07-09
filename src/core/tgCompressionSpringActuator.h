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
 * @author Drew Sabelhaus
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
 * in the hopes that this functionality will be added eventually.
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
     * Parameter defaults.
     */
    Config(
	   bool iFEA = false,
	   double s = 1000.0,
	   double d = 10.0,
	   double rL = 0.0,
	   bool moveCPA = true,
	   bool moveCPB = true);

    /**
     * List of the parameters themselves.
     * All are passed in to tgBulletCompressionSpring.
     */

    /**
     * Boolean flag for attached-ness of the free end of the spring.
     * If no, the spring only provides compression force. 
     * If yes, the spring is "attached" to both anchors, and provides a 
     * tension force too when the distance between anchors is greater 
     * than rest length.
     */
    bool isFreeEndAttached;

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
     * Control the automatic re-locating of cable anchor points.
     * 
     * When true, the default behavior, the specified attachment point
     * of a cable is moved from the actual node location (for example, at the
     * center of the circle at the end of a rod) to a location at the edge
     * of the rigid body it's attached to (for example, on the edge of the
     * circle at the end of a rod.) 
     * 
     * If unsure, leave default, because when set to 'false', the cable will
     * likely clip through the rod.
     * Disabling automatic movement of anchor points is most useful for
     * structures that need very particular cable placement.
     * Here, cable attachment point "A" corresponds to the first of the two
     * anchors of the spring-cable, or the "From" point in the tgcreator files.
     * The "B" point corresponds to the second anchor location, or the "To" point
     * in the tgcreator files.
     * 
     * These parameters are used in the createTgBulletSpringCable method inside
     * the tgBasicActuatorInfo class.
     */
    bool moveCablePointAToEdge;
    bool moveCablePointBToEdge;

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
   * Functions for interfacing with tgBulletCompressionSpring.
   * NOTE that some of these are not implemented at the moment.
   * If this class was to really become an actuator, these helper 
   * functions would probably be needed.
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
  virtual double getActuatorSpringForce();

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
