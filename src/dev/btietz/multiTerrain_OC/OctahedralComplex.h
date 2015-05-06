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

#ifndef OCTAHEDRAL_COMPLEX_H
#define OCTAHEDRAL_COMPLEX_H

/**
 * @file OctahedralComplex.h
 * @brief Implementing the cross-linked octahedral complex spine inspired by Tom Flemons
 * @author Brian Mirletz
 * @date March 2015
 * @version 1.1.0
 * $Id$
 */

#include "dev/btietz/TC_goal/BaseSpineModelGoal.h"

#include "LinearMath/btVector3.h"

class tgWorld;
class tgStructureInfo;
class tgBox;
class tgSpringCableActuator;

/**
 * This class implements the octahedral complex tensegrity spine
 * based on the work of <a href="http://www.intensiondesigns.com/models.html">Tom Flemons</a>
 */
class OctahedralComplex : public BaseSpineModelGoal
{
public: 

    OctahedralComplex(int segments, double goalAngle);

    virtual ~OctahedralComplex();
    
    virtual void setup(tgWorld& world);
    
    virtual void teardown();
        
    virtual void step(double dt);
    
    const std::vector<tgSpringCableActuator*>& getSaddleMuscles() const
    {
        return m_saddleMuscles;
    }
    
protected:
    
    std::vector<tgSpringCableActuator*> m_saddleMuscles;
    
};

#endif // FLEMONS_SPINE_MODEL_H
