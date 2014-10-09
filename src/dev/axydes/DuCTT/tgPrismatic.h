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

#ifndef TG_PRISMATIC_H
#define TG_PRISMATIC_H

/**
 * @file tgPrismatic.h
 * @brief Contains the definition of class tgPrismatic. A prismatic actuator.
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "core/tgModel.h"
#include "core/tgSubject.h"

class tgWorld;
class btSliderConstraint;

class tgPrismatic: public tgSubject<tgPrismatic>, public tgModel
{
public: 
    
    struct Config
    {
    public:
        // To make the complier happy. Probably should never be called
        Config();
        
        Config(
                double maxLength = 5,
                double minLength = 0.1,
                double maxMotorForce = 20,
                double maxVelocity = 0.01,
                double eps = 0.01
                );
        
        /**
         * Specifies the maximum length of the joint.
         * Units are meters
         */
        double m_maxLength;

        /**
         * Specifies the minimum length of the joint.
         * Units are meters
         */
        double m_minLength;

        /**
         * Specifies the maximum force that the joint motor can expend.
         */
        double m_maxMotorForce;

        /**
         * Specifies the maximum velocity of the joint motor.
         * Units are meters/second
         */
        double m_maxVelocity;

        /**
         * Specifies the maximum distance allowed between joint lengths to be
         * considered equal.
         * Units are meters
         */
        double m_eps;
    };
    
    /**
     * Constructor using tags.
     * @param[in] constraint The btSliderConstraint object that this controls and logs.
     * Set up in tgPrismaticInfo.cpp
     * @param[in] tags as passed through tgStructure and tgStructureInfo
     * @param[in] config Holds member variables like min/max length, max force
     * and velocity, and the epsilon value to use when comparing joint distance.
     */
    tgPrismatic(
        btSliderConstraint* constraint,
        const tgTags& tags,
        tgPrismatic::Config& config);
    
    /**
     * Calls teardown
     */
    virtual ~tgPrismatic();

    /**
     * Uses the Config object to set the maximum/minimum limits of the joint,
     * turns on the joint motor and sets the desired length to the minimum.
     */
    virtual void init();
    
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
     * Notifies observers of step, calls moveMotors, steps children.
     * @param[in] dt, must be >= 0.0
     */
    virtual void step(double dt);

    /**
     * Directly set the desired length of the joint, does not call moveMotors
     * @param length, desired length of the joint.
     * @return true if successfully set desired length of the joint, false if it defaulted to
     *      a maximum or minimum or failed.
     */
    virtual bool setPreferredLength(double length);

    /**
     * Directly set the maximum velocity of the motor attached
     * to the prismatic joint. Does not call moveMotors.
     * @param vel, the desired max velocity of the joint motor
     */
    void setMaxVelocity(double vel);

    /**
     * Directly set the maximum force of the motor attached
     * to the prismatic joint. Does not call moveMotors.
     * @param force, the desired max force of the joint motor
     */
    void setMaxForce(double force);

    /**
     * Called from public functions, it makes the actual length get closer
     * to m_preferredlength, according to config constraints.
     */
    virtual void moveMotors(double dt);

private:
    /**
     * A copy of the configuration POD supplied at constuction.
     * This is not const.
     */
    Config m_config;

    btSliderConstraint* m_slider;

    /**
     * Hold the desired length of the joint.
     */
    double m_preferredLength;
};

#endif // TG_PRISMATIC_H
