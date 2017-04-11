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

#ifndef SIX_BAR_MODEL_H
#define SIX_BAR_MODEL_H

/**
 * @file sixBarModel.h
 * @brief Defines a six bar 24 string tensegrity model
 * @author Edward Zhu
 * @version 1.0
 * $Id$
 */

 // This library
 #include "core/tgModel.h"
 #include "core/tgSubject.h"

// Builder libraries
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "core/tgString.h"
#include "core/abstractMarker.h"
#include "tgcreator/tgNode.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"

// C++ Standard Libraries
#include <vector>

// Forward declarations
class tgModelVisitor;
class tgStructure;
class tgWorld;

/**
 * A class that constructs a six bar tensegrity prism using the tools
 * in tgcreator. This iteration avoids using a controller and instead
 * uses the new (to v1.1) ability to define pretension in a
 * tgBasicActuator's constructor
 */
class sixBarModel : public tgSubject<sixBarModel>, public tgModel 
{
	public:
	 	
	 	/**
	     * The only constructor. Configuration parameters are within the
	     * .cpp file in this case, not passed in. 
	     */
		sixBarModel();
	    sixBarModel(int yaw, int pitch, int roll);
	    
	    /**
	     * Destructor. Deletes controllers, if any were added during setup.
	     * Teardown handles everything else.
	     */
	    virtual ~sixBarModel();

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
	    const std::vector<tgBasicActuator*>& getAllActuators() const;

	    /**
	     * Return a vector of all rod bodies for the controllers to work with.
	     * @return A vector of all of the rod rigid bodies
	     */
	    const std::vector<tgRod*>& getAllRods() const;

	    /**
	     * Return a vector of the payload body for the controllers to work with.
	     * @return A vector of the payload rigid body
	     */
	    const std::vector<tgRod*>& getPayload() const;

	    /**
	     * Return a vector of all normal vectors for the controllers to work with.
	     * @return A vector of all of the normal vectors
	     */
	    const std::vector<btVector3>& getNormVects() const;

	    /**
	     * Return a vector of all abstract markers for the controllers to work with.
	     * @return A vector of all of the abstract markers
	     */
	    const std::vector<abstractMarker>& getAllMarkers() const;

	    /**
	     * A function called during setup that rotates the structure
	     * to a face
	     * @param[in] s A tgStructure that we're building into
	     * @param[in] face The face to rotate to
	     */
	    void rotateToFace(tgStructure& s, int face);

	    void rotateYaw(tgStructure& s, double psi);

	    void rotatePitch(tgStructure& s, double theta);

	    void rotateRoll(tgStructure& s, double phi);

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
	     * the nodes based on construction parameters. This function uses the
	     * rod length parameter in the Config struct in sixBarModel.cpp to 
	     * determine the size of the tensegrity
	     * @param[in] s: A tgStructure that we're building into
	     */
	    static void addSixBarNodes(tgStructure& s);

	    /**
	     * A function called during setup that creates rods from the
	     * relevant nodes for the tensegrity.
	     * @param[in] s A tgStructure that we're building into
	     */
	    static void addSixBarRods(tgStructure& s);

	    /**
	     * A function called during setup that creates actuators from the
	     * relevant nodes for the tensegrity.
	     * @param[in] s A tgStructure that we're building into
	     */
	    static void addSixBarActuators(tgStructure& s);

	    /**
	     * A function called during setup that determines the positions of
	     * nodes of the payload in the center
	     * @param[in] s A tgStructure that we're building into
	     */
	    static void addPayloadNodes(tgStructure& s);

	    /**
	     * A function called during setup that creates rods from the
	     * relevant nodes for the payload.
	     * @param[in] s A tgStructure that we're building into
	     */
	    static void addPayloadRods(tgStructure& s);

	    /**
	     * A function called during setup that creates cables from the
	     * relevant nodes for the payload.
	     * @param[in] s A tgStructure that we're building into
	     */
	    static void addPayloadStrings(tgStructure& s);

	    /**
	     * A function called during setup that creates the tensegrity structure
	     * @param[in] s A tgStructure that we're building into
	     */
	    static void addSixBar(tgStructure& s);

	    /**
	     * A function called during setup that creates the payload structure
	     * @param[in] s A tgStructure that we're building into
	     */
	    static void addPayload(tgStructure& s);

	    // Vectors to hold actuators and rods
		std::vector<tgBasicActuator*> allActuators;
	    std::vector<tgRod*> allRods;
	    std::vector<tgRod*> payload;

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

		abstractMarker NODE0;
		abstractMarker NODE1;
		abstractMarker NODE2;
		std::vector<abstractMarker> allMarkers;

		bool motorModel;

		int yaw_init = 0;
		int pitch_init = 0;
		int roll_init = 0;
};

#endif