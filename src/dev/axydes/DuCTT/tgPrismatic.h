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
                double maxLength = 5, //todo: find better default
                double minLength = 0.1, // todo: find better default
                double maxMotorForce = 20,
                double maxVelocity = 0.5 //m/s
                );
        
        double m_maxLength;
        double m_minLength;
        double m_maxMotorForce;
        double m_maxVelocity;
    };
    
    tgPrismatic(
        btSliderConstraint* constraint,
        const tgTags& tags,
        tgPrismatic::Config& config);
    
    tgPrismatic(
        btSliderConstraint* constraint,
        std::string space_separated_tags,
        tgPrismatic::Config& config);

    virtual ~tgPrismatic();

    virtual void init();
    
    virtual void setup(tgWorld& world);

    virtual void teardown();

    virtual void step(double dt);
    
    virtual void moveMotors(double dt);

private:
    Config m_config;
    btSliderConstraint* m_slider;
};

#endif // TG_PRISMATIC_H
