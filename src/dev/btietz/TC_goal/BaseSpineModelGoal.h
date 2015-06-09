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

#ifndef BASE_SPINE_MODEL_GOAL_H
#define BASE_SPINE_MODEL_GOAL_H

/**
 * @file BaseSpineModelGoal.h
 * @brief Implementing the tetrahedral complex spine inspired by Tom Flemons
 * @author Brian Mirletz
 * @date April 2015
 * @version 1.1.0
 * $Id$
 */

#include "examples/learningSpines/BaseSpineModelLearning.h" 

#include "LinearMath/btVector3.h"

class tgWorld;
class tgStructureInfo;
class tgBox;
class tgSpringCableActuator;

/**
 * An abstract base class providing the interface for classes that attempt
 * goal directed locomtion
 */
class BaseSpineModelGoal : public BaseSpineModelLearning
{
public: 

    BaseSpineModelGoal(int segments, double goalAngle);

    virtual ~BaseSpineModelGoal();
    
    virtual void setup(tgWorld& world);
    
    virtual void teardown();
        
    virtual void step(double dt);
    
    btVector3 goalBoxPosition() const;
    
    
protected:
    
    tgBox* m_goalBox;
    
    double m_goalAngle;

};

#endif // BASE_SPINE_MODEL_GOAL_H
