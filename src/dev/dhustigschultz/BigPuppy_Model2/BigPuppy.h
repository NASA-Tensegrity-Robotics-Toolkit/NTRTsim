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

#ifndef BIGPUPPY_H
#define BIGPUPPY_H

/**
 * @file BigPuppy.h
 * @brief Implementing the Flemons quadruped model.
 * @author Dawn Hustig-Schultz
 * @date April 2015
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgModel.h" 
#include "core/tgSubject.h"
// The C++ Standard Library
#include <map>
#include <set>
#include <string>
#include <vector>

class tgSpringCableActuator;
class tgWorld;
class tgStructure;    //Do I need this, or tgStructureInfo?
class tgStructureInfo;  //Do I need this, or just tgStructure, or both?
class tgBasicActuator;  

class BigPuppy: public tgSubject<BigPuppy>, public tgModel
{
public: 

    BigPuppy();

    virtual ~BigPuppy();

    /**
     * Create the model. Place the rods and strings into the world
     * that is passed into the simulation. This is triggered
     * automatically when the model is added to the simulation, when
     * tgModel::setup(world) is called (if this model is a child),
     * and when reset is called. Also notifies controllers of setup.
     * @param[in] world - the world we're building into
     */
    virtual void setup(tgWorld& world);
    
    /**
     * Undoes setup. Deletes child models. Called automatically on
     * reset and end of simulation. Notifies controllers of teardown
     */
    virtual void teardown();
    
    /**
     * Step the model, its children. Notifies controllers of step.
     * @param[in] dt, the timestep. Must be positive.
     */
    virtual void step(double dt);

    /**
     * Return a vector of all muscles for the controllers to work with.
     * @return A vector of all of the muscles
     */
    const std::vector<tgSpringCableActuator*>& getAllActuators() const;

private:
    /**
     * A list of all of the spring cable actuators. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    std::vector<tgSpringCableActuator*> allActuators;
};

#endif
