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

#ifndef TT3_MODEL_H
#define TT3_MODEL_H

/**
 * @file TT3Model.h
 * @brief Contains the definition of class TT3Model.
 * $Id$
 */

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"
#include "core/tgRod.h"
#include "core/abstractMarker.h"
// The C++ Standard Library
#include <vector>

// Forward declarations
class tgBasicActuator;
class tgModelVisitor;
class tgStructure;
class tgWorld;

/**
 * Class that creates the six strut TT3 model created at UC Berkeley using tgcreator
 */
class TT3Model : public tgSubject<TT3Model>, public tgModel
{
public: 
	
	/**
     * The only constructor. Utilizes default constructor of tgModel
     * Configuration parameters are within the .cpp file in this case,
     * not passed in. 
     */
    TT3Model();
	
    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~TT3Model();
    
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
    void teardown();
    
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
    const std::vector<tgBasicActuator*>& getAllActuators() const;
    
    /**
     * Return a vector of all rod bodies for the controllers to work with.
     * @return A vector of all of the rod rigid bodies
     */
    const std::vector<tgRod*>& getAllRods() const;

    /**
     * Return a vector of all capsule bodies for the controllers to work with.
     * @return A vector of all of the capsule rigid bodies
     */
    const std::vector<tgRod*>& getAllCapsules() const;

    /**
         * A function called during setup that rotates the structure
         * to a face
         * @param[in] s A tgStructure that we're building into
         * @param[in] face The face to rotate to
         */
    void rotateToFace(tgStructure& s, int face);

    double rodDist;

    // Nodes at the end of each of the rods
    btVector3 node0;
    btVector3 node1;
    btVector3 node2;
    btVector3 node3;
    btVector3 node4;
    btVector3 node5;
    btVector3 node6;
    btVector3 node7;
    btVector3 node8;
    btVector3 node9;
    btVector3 node10;
    btVector3 node11;

    // Edge vectors of all closed triangles
    btVector3 face0Edge0;
    btVector3 face0Edge1;
    btVector3 face0Edge2;

    btVector3 face2Edge0;
    btVector3 face2Edge1;
    btVector3 face2Edge2;

    btVector3 face5Edge0;
    btVector3 face5Edge1;
    btVector3 face5Edge2;

    btVector3 face7Edge0;
    btVector3 face7Edge1;
    btVector3 face7Edge2;

    btVector3 face8Edge0;
    btVector3 face8Edge1;
    btVector3 face8Edge2;

    btVector3 face10Edge0;
    btVector3 face10Edge1;
    btVector3 face10Edge2;

    btVector3 face13Edge0;
    btVector3 face13Edge1;
    btVector3 face13Edge2;

    btVector3 face15Edge0;
    btVector3 face15Edge1;
    btVector3 face15Edge2;

private:
	
	/**
     * A function called during setup that determines the positions of
     * the nodes based on construction parameters. Rewrite this function
     * for your own models
     * @param[in] tetra: A tgStructure that we're building into
     */
    static void addNodes(tgStructure& s);
	
	/**
     * A function called during setup that creates rods from the
     * relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addRods(tgStructure& s);
	
	/**
     * A function called during setup that creates muscles (Strings) from
     * the relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addActuators(tgStructure& s);

    /**
     * A function called during setup that creates the tensegrity structure
     * @param[in] s A tgStructure that we're building into
     */
    static void addTT3(tgStructure& s);

private:
	
    // Vectors to hold actuators and rods
    std::vector<tgBasicActuator*> allActuators;
    std::vector<tgRod*> allRods;
    std::vector<tgRod*> allCapsules;

    // A vector to hold all normal vectors
    std::vector<btVector3> normalVectors;

    // Normal vectors of all icosahedron faces
    btVector3 face0Norm;
    btVector3 face1Norm;
    btVector3 face2Norm;
    btVector3 face3Norm;
    btVector3 face4Norm;
    btVector3 face5Norm;
    btVector3 face6Norm;
    btVector3 face7Norm;
    btVector3 face8Norm;
    btVector3 face9Norm;
    btVector3 face10Norm;
    btVector3 face11Norm;
    btVector3 face12Norm;
    btVector3 face13Norm;
    btVector3 face14Norm;
    btVector3 face15Norm;
    btVector3 face16Norm;
    btVector3 face17Norm;
    btVector3 face18Norm;
    btVector3 face19Norm;
};

#endif  // TT3_MODEL_H
