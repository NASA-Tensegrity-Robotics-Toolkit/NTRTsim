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

#ifndef PRISM_MODEL_H
#define PRISM_MODEL_H

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
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"

// C++ Standard Libraries
#include <vector>

// Forward declarations
class tgSpringCableActuator;
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
	    const std::vector<tgSpringCableActuator*>& getAllActuators() const;

	    std::vector<btRigidBody*> rodBodies;

	    double rodDist;

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

	    /**
	     * A list of all of the spring cable actuators. Will be empty until most of the way
	     * through setup when it is filled using tgModel's find methods
	     */
	    std::vector<tgSpringCableActuator*> allActuators;
};

#endif