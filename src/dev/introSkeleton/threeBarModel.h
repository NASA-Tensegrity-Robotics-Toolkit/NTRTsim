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

#ifndef THREE_BAR_MODEL_H
#define THREE_BAR_MODEL_H

/**
 * @file 3BarModel.h
 * @brief Defines a 3 strut 9 string tensegrity model
 * @author Edward Zhu
 * $Id$
 */

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"

// Builder libraries
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgNode.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"

// The C++ Standard Library
#include <vector>

// Forward declarations
class tgBasicActuator;
class tgModelVisitor;
class tgStructure;
class tgWorld;

/**
 * A class that constructs a three bar tensegrity prism using the tools
 * in tgcreator. This iteration avoids using a controller and instead
 * uses the new (to v1.1) ability to define pretension in a
 * tgBasicActuator's constructor
 */
class threeBarModel : public tgSubject<threeBarModel>, public tgModel
{
public: 
    
    /**
     * The only constructor. Configuration parameters are within the
     * .cpp file in this case, not passed in. 
     */
    threeBarModel();
    
    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~threeBarModel();
    
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
     * Receives a tgModelVisitor and dispatches itself into the
     * visitor's "render" function. This model will go to the default
     * tgModel function, which does nothing.
     * @param[in] r - a tgModelVisitor which will pass this model back
     * to itself 
     */
    virtual void onVisit(tgModelVisitor& r);
    
    /**
     * Return a vector of all actuators for the controllers to work with.
     * @return A vector of all of the actuators
     */
    std::vector<tgBasicActuator*>& getAllActuators();

    /**
     * Return a vector of all rod bodies for the controllers to work with.
     * @return A vector of all of the rod rigid bodies
     */
    std::vector<tgRod*>& getAllRods();
      
private:
    
    /**
     * A function called during setup that determines the positions of
     * the nodes based on construction parameters. Rewrite this function
     * for your own models
     * @param[in] s: A tgStructure that we're building into
     * @param[in] edge: the X distance of the base points
     * @param[in] width: the Z distance of the base triangle
     * @param[in] height: the Y distance along the axis of the prism
     */
    static void addNodes(tgStructure& s,
                            double edge,
                            double width,
                            double height);
    
    /**
     * A function called during setup that creates rods from the
     * relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addRods(tgStructure& s);
    
    /**
     * A function called during setup that creates actuators (Strings) from
     * the relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addActuators(tgStructure& s);

private:    
    /**
     * A list of all of the basic actuators. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    std::vector<tgBasicActuator*> allActuators;

    /**
     * A list of all of the rods. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    std::vector<tgRod*> allRods;
};

#endif  // THREE_BAR_MODEL_H
