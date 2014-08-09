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

#ifndef TG_RB_STRING_H
#define TG_RB_STRING_H

/**
 * @file tgPrismatic.h
 * @brief Contains the definition of class tgPrismatic. A prismatic actuator.
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "core/tgBulletUtil.h"
#include "core/tgCast.h"
#include "core/tgModel.h" 
#include "core/tgRod.h"
#include "core/tgSubject.h"
#include "core/tgWorldBulletPhysicsImpl.h"

// The C++ Standard Library
#include <cmath>
#include <stdexcept>

#include <set>
#include <map>

class tgWorld;

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
                std::size_t segments = 2
                );
        
        double m_maxLength;
        double m_minLength;
        double m_maxMotorForce;
        std::size_t m_segments;
    };
    
    tgPrismatic(
            btRigidBody* body1,
            btVector3 pos1,
            btRigidBody* body2,
            btVector3 pos2,
            const tgTags& tags,
           tgPrismatic::Config& config);
    
    tgPrismatic(
            btRigidBody* body1,
            btVector3 pos1,
            btRigidBody* body2,
            btVector3 pos2,
            std::string space_separated_tags,
           tgPrismatic::Config& config);

    virtual ~tgPrismatic();// {}

    virtual void init();
    
    virtual void setup(tgWorld& world);

    virtual void teardown();

    virtual void step(double dt);
    
    virtual void moveMotors(double dt);

    const int getSegments() const
    {
        return m_config.m_segments;
    }

private:
    std::vector<tgRod*> allSegments;

    Config m_config;
    btRigidBody* m_body1;
    btRigidBody* m_body2;
    btVector3 m_pos1;
    btVector3 m_pos2;

    btSliderConstraint* m_slider;
    btSoftRigidDynamicsWorld* dynWorld;
};

#endif // TG_RB_STRING_TEST_H
