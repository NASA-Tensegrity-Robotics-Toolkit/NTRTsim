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

#ifndef TG_LINEAR_STRING_H
#define TG_LINEAR_STRING_H

/**
 * @file tgLinearString.h
 * @brief Contains the definition of class tgLinearString.
 * @author Brian Tietz
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

// This application
#include "tgModel.h"
#include "tgSpringCableActuator.h"

// Forward declarations
class tgBulletSpringCable;
class tgModelVisitor;
class tgWorld;

// Should always be a child Model of a tgModel
class tgLinearString : public tgSpringCableActuator
{
public: 

    /**
     * Constructor using tags. Typically called in tgLinearStringInfo.cpp 
     * @param[in] muscle The muscle2P object that this controls and logs.
     * Set up in tgLinearStringInfo.cpp
     * @param[in] tags as passed through tgStructure and tgStructureInfo
     * @param[in] config Holds member variables like elasticity, damping
     * and motor parameters. See tgSpringCableActuator
     * @param[in] hist whether or not to log additional history @todo 
     * move hist to config
     */    
    tgLinearString(tgBulletSpringCable* muscle,
           const tgTags& tags,
           tgSpringCableActuator::Config& config);
    
    /**
     * Destructor deletes the tgBulletSpringCable
     */
    virtual ~tgLinearString();
    
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
	 * Set the relevant control variable for this class, such as 
	 * commanded position or torque
	 */
	virtual void setControlInput(double input);
	
	/**
	 * Secondary function for those classes which need to know how much
	 * time has elapsed since they last recieved input (such as 
	 * tgBasicActuator's moveMotors(dt) function)
	 * This will silently fail if it is called erroneously
	 */
	virtual void setControlInput(double input, double dt);
        
    /**
     * Directly set m_preferredLength (see base class tgSpringCableActuator)
     * Calls moveMotors(dt) to adjust the rest length of tgBulletSpringCable
     * @todo Refactor to setControlInput
     */
    void setRestLength(double newLength, float dt);
	
    /**
     * Directly set m_preferredLength (see base class tgSpringCableActuator)
     * Does not call moveMotors.
     * @todo Refactor to setControlInput
     */
    void setPrefLength(double newLength);

    /**
     * Directly set m_preferredLength (see base class tgSpringCableActuator)
     * Directly adjusts the rest length of tgBulletSpringCable, not using moveMotors.
     * @todo Remove this - config can handle it now
     */
    void setRestLengthSingleStep(double newLength);
    
    virtual const tgSpringCableActuator::SpringCableActuatorHistory& getHistory() const;
    
    
    /** Called from public functions, it makes the restLength get closer
     * to preferredlength, according to config constraints.
     */
    virtual void moveMotors(double dt);


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
    
};


#endif
