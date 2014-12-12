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

#ifndef SRC_CORE_TG_IMPEDANCECONTROLLER_H_
#define SRC_CORE_TG_IMPEDANCECONTROLLER_H_

/**
 * @file tgImpedanceController.h
 * @brief Contains the definition of class ImpedanceControl.
 * $Id$
 */

// Forward references
class tgBasicController;
class tgBasicActuator;
/**
 * Influences the spring-cable actuator tension using length and velocity.
 */
class tgImpedanceController
{
 public:
     
     ///Constructors
     /**
     * The null constructor sets defaults values for the
     * member variables.
     */
        tgImpedanceController();

    /**
     * This constructor supplies initial values for all member
     * variables.
     * @param[in] offsetTension the initial value for the offset
     * tension property; must be non-negative
     * @param[in] lengthStiffness the initial value for the length
     * stiffness property; must be non-negative
     * @param[in] velStiffness the initial value for the velocity
     * stiffness property; must be non-negative
     */
    tgImpedanceController(double offsetTension,
             double lengthStiffness,
             double velStiffness);
    
    ///Control Functions

    /**
     * @param[in] Control a tgBasicActuator given a new offset force and a
     * new offset velocity.
     * @param[in] mString a pointer to a tgBasicActuator; must not be NULL
     * @param[in] deltaTimeSeconds the number of seconds since the
     * last call
     * @param[in] newPosition the current position of the Muscle
     * @todo should we add an offset position so it can just control??
     */
    double control(tgBasicController& mLocalController,
                    double deltaTimeSeconds,
                    double newPosition,
                    double offsetVel = 0);
    
                    
    double controlTension(tgBasicController& mLocalController,
                    double deltaTimeSeconds,
                    double newPosition,
                    double offsetTension,
                    double offsetVel = 0);
    /**
     * For those who control on tension without PID
     */
    double control(tgBasicActuator& mLocalController,
                    double deltaTimeSeconds,
                    double newPosition,
                    double offsetVel = 0);
    
                    
    double controlTension(tgBasicActuator& mLocalController,
                    double deltaTimeSeconds,
                    double newPosition,
                    double offsetTension,
                    double offsetVel = 0);
    /**
     * Set the value of the offset tension property.
     * @param[in] the new value for the offset tension property
     * must be non-negative
     */
    void setOffsetTension(double offsetTension);

    /**
     * Set the value of the length stiffness property.
     * @param[in] the new value for the length stiffness property
     * must be non-negative
     */
    void setLengthStiffness(double lengthStiffness);

    /**
     * Set the value of the velocity stiffness property.
     * @param[in] the new value for the velocity stiffness property
     * must be non-negative
     */
    void setVelStiffness(double velStiffness);

    /**
     * Return the value of the offset tension property.
     * @return the value of the offset tension property
     */
    double getOffsetTension() const
    {
            return m_offsetTension;
    }

    /**
     * Return the value of the length stiffness property.
     * @return the value of the length stiffness property
     */
    double getLengthStiffness() const
    {
            return m_lengthStiffness;
    }

    /**
     * Return the value of the velocity stiffness property.
     * @return the value of the velocity stiffness property
     */
    double getVelStiffness() const
    {
            return m_velStiffness;
    }
        
 protected:

    /**
     * A force component.
     * Must be non-negative to ensure stability.
     * The units are application-dependent, e.g., Newtons or
     * centinewtons.
     */
    double m_offsetTension;

    /**
     * Used to determine the force component attributable to a
     * Muscle's length.
     * Must be non-negative to ensure stability.
     * The units are kg/sec^2. When multiplied by the cable length
     * it gives a force component.
     */
    double m_lengthStiffness;

    /**
     * Used to determine the force component attributable to a
     * Muscle's velocity.
     * The units are kg/sec. When multiplied by the cable velocity
     * it gives a force component.
     * Must be non-negative to ensure stability
     */
    double m_velStiffness;

 private:
        
    /** Integrity predicate. */
    bool invariant() const;
};

#endif  // SRC_CORE_IMPEDANCECONTROL_H_
