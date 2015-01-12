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

#ifndef MUSCLE_NP_CONS
#define MUSCLE_NP_CONS

/**
* @file MuscleNPCons.h
* @brief For testing MuscleNP contacts
* @author Brian Mirletz
* @version 1.0.0
* $Id$
*/

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"

// The Bullet Physics Library
#include "LinearMath/btVector3.h"

// The C++ Standard Library
#include <vector>

// Forward declarations
class tgSpringCableActuator;
class tgModelVisitor;
class tgWorld;
class tgBaseRigid;

/**
* A class that constructs a three bar tensegrity prism using the tools
* in tgcreator.
*/
class MuscleNPCons : public tgSubject<MuscleNPCons>, public tgModel
{
public:
	/**
	* The only constructor. Configuration parameters are within the
	* .cpp file in this case, not passed in.
	*/
	MuscleNPCons();
	/**
	* Destructor. Deletes controllers, if any were added during setup.
	* Teardown handles everything else.
	*/
	virtual ~MuscleNPCons();
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
	virtual void onVisit(const tgModelVisitor& r) const;
	
		/**
	 * Sums the square of the velocity times the mass
	 * @todo account for rotational energy as well, and energy in the string
	 */
	virtual double getEnergy() const;
	
	/**
	 * Returns a btVector3 of the current center of mass velocity (momentum)
	 */
	virtual btVector3 getMomentum() const;
	
	virtual btVector3 getVelocityOfBody(int body_num) const;
	
private:
	double totalTime;
	std::vector<tgSpringCableActuator*> allMuscles;
	std::vector<tgBaseRigid*> allRods;
	
};
#endif // SIMPLE_CORDE_TENSEGRITY_H
