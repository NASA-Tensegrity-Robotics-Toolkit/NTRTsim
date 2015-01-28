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

#ifndef TETRA_SPINE_KINEMATIC
#define TETRA_SPINE_KINEMATIC

/**
 * @file TetraSpineKinematic.h
 * @brief Middle segment of In Won's robot reconfigured with tgBulletContactSpringCable and more segments
 * @author Brian Mirletz
 * @date January 2015
 * @version 1.1.0
 * $Id$
 */

// This library
#include "examples/learningSpines/BaseSpineModelLearning.h"

// Forward Declarations
class tgWorld;


/**
 * Basically the same structure as NestedStructureTestModel
 * just with different parameters and learning capabilities.
 * This is an simplification of the model used in the 2015 ICRA paper
 */
class TetraSpineKinematic: public BaseSpineModelLearning
{
public: 

    
    TetraSpineKinematic(size_t segments);

    virtual ~TetraSpineKinematic();
    
    virtual void setup(tgWorld& world);
    
    virtual void teardown();
    
    virtual void step(const double dt);

};

#endif
