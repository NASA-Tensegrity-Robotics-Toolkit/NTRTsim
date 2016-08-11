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

#ifndef SRC_CORE_TG_SPRING_CABLE_ACTUATOR_H
#define SRC_CORE_TG_SPRING_CABLE_ACTUATOR_H

/**
 * @file tgSpringCableActuator.h
 * @brief Contains the definition of abstract base class tgSpringCableActuator.
 * Assumes that the string is linear (F = -kX - bV)
 * @author Brian Tietz
 * $Id$
 */

// This application
#include "tgModel.h"
#include "tgControllable.h"
#include "tgSubject.h"

#include <deque> // For history
// Forward declarations
class tgWorld;
class tgSpringCable;

/**
 * Sets a basic API for spring cable actuator models, so controllers can interface
 * with all of them the same way. Abstract since it doesn not define
 * tgControllable's setControlInput.
 */
// Should always be a child Model of a tgModel
class tgSpringCableActuator : public tgModel, public tgControllable,
                                public tgSubject<tgSpringCableActuator>
{
public: 
    
    struct Config
    {
    public:
        /**
         * The only constructor. Parameters set the defaults for string
         * material properties, construction properties and the motor model.
         * Individual parameters are discussed below.
         */
        Config( double s = 1000.0,
        double d = 10.0,
        double p = 0.0,
        bool h = false,
        double mf = 1000.0,
        double tVel = 100.0,
        double mnAL = 0.1,
        double mnRL = 0.1,
	double rot = 0,
	bool moveCPA = true,
	bool moveCPB = true);
      
      /**
       * Scale parameters that depend on the length of the simulation.
       * Centemeter scale has been used throughout many of the demos,
       * so those assumptions are baked into the default parameters.
       */  
      void scale (double sf);
      
      // Muscle2P Parameters
      /**
       * Linear Hookean stiffness of the string (k). Must be non-negative.
       * Upper limit depends on the timestep - stiffer springs will
       * require a lower timestep. Used in Muscle2P, tensionMinLengthController,
       * moveMotors (to enforce max force constraints) 
       * Units are mass / seconds^2
       */
      double stiffness;
      /**
       * Specifies the damping (b) term in the linear force equation.
       * Units are mass / seconds
       * Must be non-negative.  Similar to stiffness in usage
       */
      double damping;
      
      /**
       * Specifies the amount of tension (in units of force) in the
       * string at construction. Adjusts the rest length,
       * Must be low enough (based on stiffness) to
       * leave the rest length non-negitive
       */
      double pretension;
      
      // History Parameters
      /**
       * Specifies whether data such as length and tension will be stored
       * in deque objects. Useful for computing the energy of a trial.
       */
      bool hist;
              
      // Motor model parameters
      /**
       * The parameters that affect how the string can be controlled.
       * Enforces a flat torque (force) speed curve with tgBasicActuator, and 
       * a linear torque speed curve with tgKinematicActuator
       */
      
      /**
       * Maximum tension that the motor can exert. String will not shorten
       * if this threshold is exceeded, but the tension can still be
       * increased by other members.
       * Units are force (length * mass / seconds^2)
       * Must be nonnegative.
       */
      double maxTens;
      
      /**
       * Maximum velocity of the motor, usage in moveMotors
       * Units are length/seconds
       * Must be nonnegative.
       */
      double targetVelocity;
      
      /**
       * Actual length below which motor ceases to shorten.
       * Units are length
       * Must be nonnegative
       */
      double minActualLength;
      
      /**
       * Rest length below which motor ceases to shorten
       * Units are length
       * Must be nonnegative
       */
      double minRestLength;
      
      // Construction Parameters
      /**
       * Specifies the rotation around the face of the object its attached
       * to.  +/- PI is the most meaningful. absolute value must be < 2 * PI
       * Units are radians.
       * @todo Is this meaningful for non-rod shapes?
       */
      double rotation;

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
    
    /** Encapsulate the history members. */
    struct SpringCableActuatorHistory
    {
        /** Length history. */
        std::deque<double> lastLengths;
        
        /** Rest length history. */
        std::deque<double> restLengths;

        /** Damping history. */
        std::deque<double> dampingHistory;

        /** Velocity history. */
        std::deque<double> lastVelocities;
        
        /** Tension history. */
        std::deque<double> tensionHistory;
    };

    /** Deletes history and spring cable instantiation */
    virtual ~tgSpringCableActuator();
    
    /** Just calls tgModel::setup(world) - sets up any children */
    virtual void setup(tgWorld& world);
    
    /** Just calls tgModel::teardown(world) - sets up any children */
    virtual void teardown();
    
    /** Just calls tgModel::step(dt) - steps any children */
    virtual void step(double dt);
    
    /**
     * Functions for interfacing with tgSpringCable
     */
       
    /**
     * Returns the length of the spring cable when the simulation
     * started
     */
    virtual const double getStartLength() const;
    
    /**
     * Returns the current length of the spring cable
     */
    virtual const double getCurrentLength() const;
    
    /**
     * Returns the current tension in the spring. Equal to
     * stiffness * (current length - rest length)
     */
    virtual const double getTension() const;
    
    /**
     * Returns the rest length of the spring-cable
     */
    virtual const double getRestLength() const;
    
    /**
     * In the default implementation returns the change in actual
     * length / time of the spring cable. Some child classes return
     * the actuator velocity
     */
    virtual const double getVelocity() const;
    
    /**
     * Return a const reference to the history struct
     */
    virtual const tgSpringCableActuator::SpringCableActuatorHistory& getHistory() const;
    
    /**
     * Returns a pointer the string's tgBulletSpringCable. Used for rendering in
     * tgBulletRenderer
     */
    const tgSpringCable* getSpringCable() const
    {
      return m_springCable;
    }
    
    /**
     * Return a const reference to m_config for scaling values in
     * neural networks
     */
    const Config& getConfig() const
    {
        return m_config;
    }
    
protected: 
    
    /**
     * Need to pass tags down to tgModel, but these should only be 
     * called by sub classes
     */    
    tgSpringCableActuator(tgSpringCable* springCable,
			const tgTags& tags,
           tgSpringCableActuator::Config& config);
           
protected:
    /** The tgSpringCable system this actuator acts upon */
    tgSpringCable* m_springCable;

    /**
     * A copy of the configuration POD supplied at constuction.
     * This is not const.
     */
    Config m_config;
    
    /** All history sequences. */
    SpringCableActuatorHistory * const m_pHistory;

    
     /**
     * Motor Model Parameters
     */
    
    /**
     * The current rest length of the spring cable. Directly sets the
     * m_restLength parameter of m_springCable in the appropreate child
     * class functions
     */
    double m_restLength;
    

    /**
     * Tracking the start length to avoid using deques when history is
     * off.
     */
    double m_startLength;
    
    /**
     * Tracking the most recent velocity to avoid using deques when
     * history is off.
     */
    double m_prevVelocity;
private:

    /**
     * Helper function to perform what is in common to all constructor bodies.
     */
    void constructorAux();

    /** Integrity predicate. */
    bool invariant() const;


};


#endif // SRC_CORE_TG_SPRING_CABLE_ACTUATOR_H
