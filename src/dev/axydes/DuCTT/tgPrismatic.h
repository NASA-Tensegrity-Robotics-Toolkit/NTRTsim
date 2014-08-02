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

#include "core/tgCast.h"
#include "core/tgModel.h" 
#include "core/tgRod.h"
#include "core/tgSubject.h"

// The C++ Standard Library
#include <cmath>
#include <stdexcept>

#include <set>
#include <map>

class tgWorld;

class tgPrismatic: public tgModel
{
public: 
    
    struct Config
    {
    public:
        // To make the complier happy. Probably should never be called
        Config();
        
        Config( std::size_t segments,
                const tgRod::Config& rod1Conf,
                const tgRod::Config& rod2Conf,
                double minTotalLength = 0.1); // todo: find better default
        
        std::size_t m_segments;
        tgRod::Config m_rod1Config;
        tgRod::Config m_rod2Config;
        double m_minTotalLength;
    };
    
    tgPrismatic(const tgTags& tags,
           tgPrismatic::Config& config);
    
    tgPrismatic(std::string space_separated_tags,
           tgPrismatic::Config& config);

    virtual ~tgPrismatic() {}
    
    /** @todo Get rid of this. */
    virtual void setup(tgWorld& world);
    
    // @todo: Is there a way that we can get rid of the need to override this function in subclasses of tgModel? 
    // comment_BRT: only if we can somehow make tgModel a template class,
    // we need to know what class we're notifying
    virtual void step(double dt);
    
    virtual void teardown();
    
    virtual void moveMotors(double dt);

    const int getSegments() const
    {
        return m_config.m_segments;
    }

private:
    std::vector<tgRod*> allSegments;

    Config m_config;
};

#endif // TG_RB_STRING_TEST_H
