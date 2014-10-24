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

#ifndef DUCTT_MODEL_H
#define DUCTT_MODEL_H

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
// The C++ Standard Library
#include <vector>

// Forward declarations
class tgModelVisitor;
class tgStructure;
class tgWorld;
class tgPrismatic;
class tgLinearString;
class PretensionController;

/**
 * A class that constructs a three bar tensegrity DuCTT using the tools
 * in tgcreator.
 */
class DuCTTRobotModel : public tgSubject<DuCTTRobotModel>, public tgModel
{
public: 
    
    /**
     * The only constructor. Configuration parameters are within the
     * .cpp file in this case, not passed in. 
     */
    DuCTTRobotModel();
    
    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~DuCTTRobotModel();
    
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
     * Return a vector of all muscles for the controllers to work with.
     * @return A vector of all of the muscles
     */
    const std::vector<tgLinearString*>& getAllMuscles() const;

    /**
     * Return a vector of all prismatic joints for the controllers to work with.
     * @return A vector of all of the prismatic joints
     */
    const std::vector<tgPrismatic*>& getAllPrismatics() const;
private:
    
    /**
     * A function called during setup that determines the positions of
     * the nodes based on construction parameters. Rewrite this function
     * for your own models
     * @param[in] tetra: A tgStructure that we're building into
     * @param[in] edge: the X distance of the base points
     * @param[in] width: the Z distance of the base triangle
     * @param[in] height: the Y distance along the axis of the DuCTT
     */
    static void addNodes(tgStructure& tetra,
                            double edge,
                            double width,
                            double height,
                            double distBtHinges = 0.1,
                            double distBtNodes = 0.1);

    /**
     * A function called during setup that creates rods from the
     * relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     * @param[in] startNode Number of node to start with
     */
    static void addRods(tgStructure& s, int startNode = 0);
    
    /**
     * A function called during setup that creates muscles (Strings) from
     * the relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addMuscles(tgStructure& s, int topNodesStart);

private:
    /**
     * A controller that is attached to all of the strings which applies
     * pretension to the muscles.
     */
    PretensionController* m_pStringController;
    
    /**
     * A list of all of the muscles. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    std::vector<tgLinearString*> allMuscles;

    /**
     * A list of all of the prismatic joints. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    std::vector<tgPrismatic*> allPrisms;
};

#endif  // DuCTT_MODEL_H
