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

/**
 * @file tgPrismatic.cpp
 * @brief Contains the definition of class tgPrismatic. A prismatic actuator.
 * @author Alexander Xydes
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "tgPrismatic.h"

/**
 * Dummy constructor to make the compiler happy.
 * @todo remove this
 */
tgPrismatic::Config::Config()
{
    throw std::invalid_argument("Failed to provide arguments to tgPrismatic::Config");
}

tgPrismatic::Config::Config(
                const tgRod::Config& rodConf,
                double minTotalLength,
                std::size_t segments
        ) :
m_segments(segments),
m_rodConfig(rodConf),
m_minTotalLength(minTotalLength)
{
}

tgPrismatic::tgPrismatic(const tgTags& tags,
           tgPrismatic::Config& config) :
m_config(config)
{
}

tgPrismatic::tgPrismatic(std::string space_separated_tags,
           tgPrismatic::Config& config) :
m_config(config)
{
}

void tgPrismatic::setup(tgWorld& world)
{
    std::cout << "tgPrismatic::setup() 1" << std::endl;
    // These occur after the build process
    allSegments = tgCast::filter<tgModel, tgRod> (getDescendants());

    std::cout << "tgPrismatic::setup() Num segments: " << allSegments.size() << std::endl;
//    assert(allSegments.size() == m_config.m_segments);

    // All the heavy lifting is done by info
    tgModel::setup(world);
}

void tgPrismatic::teardown()
{
    tgModel::teardown();
}

void tgPrismatic::step(double dt)
{
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive.");
    }
    else
    { 
        tgModel::step(dt);  // Step any children
    }
}

void tgPrismatic::moveMotors(double dt)
{
}
