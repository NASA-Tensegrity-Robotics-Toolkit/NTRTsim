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
 * @file tgRBString.h
 * @brief Contains the definition of class tgRBString. A string with
 * small rigid bodies to create contact dynamics. Depricated as of
 * version 1.1.0
 * @author Brian Tietz
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

#include "core/tgModel.h" 

#include "core/tgSubject.h"

#include "core/tgBaseString.h"
#include "core/tgRod.h"

#include <set>
#include <map>

class tgWorld;
class tgLinearString;

class tgRBString: public tgBaseString
{
public: 
    
    struct Config
    {
    public:
        // To make the complier happy. Probably should neber be called
        Config();
        
        Config( std::size_t segments,
                const tgRod::Config& rodConf,
                const tgBaseString::Config& stringConf,
                double minTotalLength = 0.1); // todo: find better default
        
        std::size_t m_segments;
        tgRod::Config           m_rodConfig;
        tgBaseString::Config  m_stringConfig;
        double m_minTotalLength;
    };
    
    tgRBString(const tgTags& tags,
           tgRBString::Config& config,
           double restLength);

    virtual ~tgRBString() {}
    
    /** @todo Get rid of this. */
    virtual void setup(tgWorld& world);
    
    // @todo: Is there a way that we can get rid of the need to override this function in subclasses of tgModel? 
    // comment_BRT: only if we can somehow make tgModel a template class,
    // we need to know what class we're notifying
    virtual void step(double dt);
    
    void changeMuscles (double lengthPercent, double dt);
    
    virtual void teardown();
    
    virtual void moveMotors(double dt);
    
    virtual void tensionMinLengthController(const double targetTension,
                                            float dt);
    
    virtual const double getStartLength() const;
    
    /* May want to calculate the point to point distance between 
    * connection points. Need to store a lot more things to do
    * that.
    */
    
    virtual const double getCurrentLength() const;
    
    virtual const double getTension() const;
    
    virtual const double getRestLength() const;
    
    virtual const double getVelocity() const;
    
    const int getSegments() const
    {
        return m_config.m_segments;
    }

private:
    
    const double computeVelocity(const double dt) const;
    
    void logHistory(const double dt);

private:
    std::vector<tgLinearString*> allMuscles;
    std::vector<tgRod*> allSegments;
    
    Config m_config;
    
    // Compute at setup based on number of segments and stiffness
    // If setup can be eliminated this can be const
    double m_effectiveStiffness;
    
    bool history;
};

#endif // TG_RB_STRING_TEST_H
