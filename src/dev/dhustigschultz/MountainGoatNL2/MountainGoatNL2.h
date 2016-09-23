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

#ifndef MOUNTAIN_GOAT_NL_2_H
#define MOUNTAIN_GOAT_NL_2_H

/**
 * @file MountainGoatNL2.h
 * @brief Trying out some new legs for MountainGoat
 * @author Dawn Hustig-Schultz
 * @date Aug 2016
 * @version 1.0.0
 * $Id$
 */

#include "dev/dhustigschultz/BigPuppy_SpineOnly_Stats/BaseQuadModelLearning.h"

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
class tgStructure;    
class tgStructureInfo;  
class tgBasicActuator;  

class MountainGoatNL2: public BaseQuadModelLearning 
{
public: 

    MountainGoatNL2(int segments, int hips, int legs);

    virtual ~MountainGoatNL2();

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

protected: 

    const std::size_t m_legs;

private:

     /** 
    * A function called during setup that determines the positions of
    * the nodes of the quadruped's leg based on construction parameters. Rewrite this function
    * for your own models
    * @param[in] s: A tgStructure that we're building into
    * r: one half the height of the tgStructure
    */
    void addNodesLeg(tgStructure& s, double r); 

    /** 
    * A function called during setup that creates the rods for the quadruped's leg from
    * the relevant nodes. Rewrite this function for your own models.
    * @param[in] s: A tgStructure that we're building into
    */ 
    static void addRodsLeg(tgStructure& s); 

     /** 
    * A function called during setup that determines the positions of
    * the nodes of the quadruped's hip based on construction parameters. Rewrite this function
    * for your own models
    * @param[in] s: A tgStructure that we're building into
    * r: one half the height of the tgStructure
    */
    void addNodesHip(tgStructure& s, double r); 

    /** 
    * A function called during setup that creates the rods for the quadruped's hip from
    * the relevant nodes. Rewrite this function for your own models.
    * @param[in] s: A tgStructure that we're building into
    */
    static void addRodsHip(tgStructure& s); 

     /** 
    * A function called during setup that determines the positions of
    * the nodes of a single vertebra (which is an X-tensegrity module) 
    * based on construction parameters. Rewrite this function
    * for your own models
    * @param[in] s: A tgStructure that we're building into
    * r: one half the height of the tgStructure
    */
    void addNodesVertebra(tgStructure& s, double r);

    /** 
    * A function called during setup that creates the rods of a single 
    * vertebra (which is an X-tensegrity module) from
    * the relevant nodes. Rewrite this function for your own models.
    * @param[in] s: A tgStructure that we're building into
    */
    static void addRodsVertebra(tgStructure& s); 

    /**
    * A function called during setup that adds all the segments to the 
    * BigPuppy structure. Rewrite this function for your own models. 
    * @param[in] puppy: A tgStructure that we're building into
    * r: one half the height of a substructure tgStructure
    * segments: the number of segments in the spine of the quadruped
    * hips: the number of hip segments
    * legs: the number of leg segments
    * feet: the number of feet segments
    */
    void addSegments(tgStructure& puppy, tgStructure& vertebra, tgStructure& hip, tgStructure& leg, double r); 

     /**
    * A function called during setup that creates muscles (Strings) for the quadruped from
    * the relevant nodes. Rewrite this function for your own models.
    * @param[in] puppy: A tgStructure that we're building into
    */
    void addMuscles(tgStructure& puppy); 

};

#endif
