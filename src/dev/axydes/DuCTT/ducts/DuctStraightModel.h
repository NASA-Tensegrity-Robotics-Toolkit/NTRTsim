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

#ifndef DUCT_STRAIGHT_MODEL_H
#define DUCT_STRAIGHT_MODEL_H

/**
 * @file DuCTTModel.h
 * @brief Defines a 3 strut 9 string tensegrity model
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <vector>

// Forward declarations
class tgModelVisitor;
class tgStructure;
class tgWorld;
class tgBox;

/**
 * A class that constructs a straight rectangular duct using the tools
 * in tgcreator.
 */
class DuctStraightModel : public tgModel
{
public: 
    /**
     * Configuration parameters so they're easily accessable.
     * All parameters must be positive.
     //
     // see tgBaseString.h for a descripton of some of these rod parameters
     // (specifically, those related to the motor moving the strings.)
     //
     // NOTE that any parameter that depends on units of length will scale
     // with the current gravity scaling. E.g., with gravity as 981,
     // the length units below are in centimeters.
     //
     // Total mass of bars is about 1.5 kg.  Total
     */
    struct Config
    {
        Config(
            double ductHeight = 40,
            double ductWidth = 40,
            double distance = 100,
            double wallWidth = 0.5,
//            double friction = 1.0,
            double friction = 10.0,
            int axis = 1,
            btVector3 startPos = btVector3(0,0,0)
        );

        double m_ductHeight;
        double m_ductWidth;
        double m_distance; //amount of distance to extend
        double m_wallWidth;
        double m_friction;
        int m_axis; // which axis to extend along, defaults to y, 0=x, 1=y, 2=z
        btVector3 m_startPos;
    };

    /**
     * The only constructor. Configuration parameters are within the
     * .cpp file in this case, not passed in. 
     */
    DuctStraightModel();
    DuctStraightModel(DuctStraightModel::Config &config);
    
    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~DuctStraightModel();

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
private:
    /**
     * A function called during setup that determines the positions of
     * the nodes based on construction parameters. Rewrite this function
     * for your own models
     * @param[in] s: A tgStructure that we're building into
     */
    void addNodes(tgStructure& s);

    void addNodesXAxis(tgStructure &s);
    void addNodesYAxis(tgStructure &s);
    void addNodesZAxis(tgStructure &s);

    /**
     * Adds tags to the node pairs for the boxes of the duct.
     * @param[in] s: A tgStructure that we're building into
     */
    void addBoxes(tgStructure& s);

    DuctStraightModel::Config m_config;
};

#endif  // DuCTT_MODEL_H
