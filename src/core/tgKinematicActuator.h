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

#ifndef SRC_CORE_TG_KINEMATIC_ACTUATOR_H
#define SRC_CORE_TG_KINEMATIC_ACTUATOR_H

/**
 * @file tgKinematicActuator.h
 * @brief Contains the definition of class tgKinematicActuator.
 * @author Brian Mirletz
 * @date Dec 2014
 * $Id$
 */

// This application
#include "core/tgModel.h"
#include "core/tgSpringCableActuator.h"

// Forward declarations
class tgBulletSpringCable;
class tgModelVisitor;
class tgWorld;

// Should always be a child Model of a tgModel
class tgKinematicActuator : public tgSpringCableActuator
{
public: 
	struct Config : public tgSpringCableActuator::Config
	{
		Config(double s = 1000.0,
				double d = 10.0,
				double p = 0.0,
				double rad = 1.0,
				double moFric = 0.0,
				double moInert = 1.0,
				bool back = false,
				bool h = false,
				double mf = 1000.0,
				double tVel = 100.0,
				double mnAL = 0.1,
				double mnRL = 0.1,
				double rot = 0);
		
		/**
		 * Scale parameters that depend on the length of the simulation.
		 * Centemeter scale has been used throughout many of the demos,
		 * so those assumptions are baked into the default parameters.
		*/  
        void scale (double sf);
		
		/**
		 * Units of length
		 */
		double radius;
		
		/**
		 * This has units of length^2 * mass / sec as it gets
		 * multiplied by speed when added to d-omega/dt
		 * Therefore this is really a damping coefficent (B) not
		 *  the unitless (mu)
		 */
		double motorFriction;
		
		/**
		 * This has units of mass * length^2. If this and r are
		 * both 1 its effectively a linear actuator
		 */
		double motorInertia;
		
		/**
		 * Should probably have friction if this is true
		 */
		bool backdrivable;
		/**
		 * Convience values calculated from other values
		 */
		double maxOmega;
		double maxTorque;
	};
	
    /**
     * Constructor using tags. Typically called in tgKinematicActuatorInfo.cpp 
     * @param[in] muscle The muscle2P object that this controls and logs.
     * Set up in tgKinematicActuatorInfo.cpp
     * @param[in] tags as passed through tgStructure and tgStructureInfo
     * @param[in] config Holds member variables like elasticity, damping
     * and motor parameters. See tgSpringCableActuator
     */    
    tgKinematicActuator(tgBulletSpringCable* muscle,
           const tgTags& tags,
           tgKinematicActuator::Config& config);
    
    /**
     * Destructor deletes the tgBulletSpringCable
     */
    virtual ~tgKinematicActuator();
    
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
     * tgBulletSpringCable, logs history if desired, steps children.
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
     * Functions for interfacing with muscle2P, and higher level controllers
     */
     
    /**
     * Return the appropreate values.
     */
    /**
     * Returns the linearized velocity of the motor, as opposed to 
     * tgBasicActuator which returns the velocity of the muscle material
     */
    virtual const double getVelocity() const;
    
    virtual const tgSpringCableActuator::SpringCableActuatorHistory& getHistory() const;
    
    /**
     * Applies a linear torque-speed function to restrict the 
     * available speeds and torques
     */
    virtual double getAppliedTorque(double desiredTorque) const;
	
	
	/**
	 * Set the value of m_desiredTorque for this timestep.
	 * Value will be scaled by getAppliedTorque, so we don't have to
	 * check it here.
	 */
	virtual void setControlInput(double input);
	
protected:
	
	virtual void integrateRestLength(double dt);
private:

    /**
     * Helper function to perform what is in common to all constructor bodies.
     */
    void constructorAux();

    /**
     * Append damping, rest length and tension values to the history member
     * variables.
     */
    void logHistory();

    /** Integrity predicate. */
    bool invariant() const;
    
    /**
     * Hold the previous value so history can be turned off
     */
    double prevVel;
    
    /**
     * Units of rad/sec
     */
    double m_motorVel;
    
    /**
     * Units of rad/sec^2
     */
    double m_motorAcc;
	
	/**
	 * Units of torque (length^2 * mass / sec^2)
	 */
	double m_desiredTorque;
    
    double m_appliedTorque;
    
    /**
     * Override the base config to get the extra parameters
     */
    tgKinematicActuator::Config m_config;
    
};


#endif
