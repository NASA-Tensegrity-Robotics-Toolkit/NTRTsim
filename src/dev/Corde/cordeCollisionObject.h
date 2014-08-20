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

#ifndef CORDE_COLLISION_OBJECT
#define CORDE_COLLISION_OBJECT

/**
 * @file CordeCollisionObject.h
 * @brief Interface Between Corde Model and Bullet
 * @author Brian Mirletz
 * $Id$
 */

// Corde Physics
#include "CordeModel.h"

// Bullet Physics
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

// Bullet Linear Algebra
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

// The C++ Standard Library
#include <vector>

class cordeSolver;
class btCollisionObjectWrapper;

class cordeCollisionObject : public CordeModel, public btCollisionObject
{
public:

	cordeCollisionObject(std::vector<btVector3>& centerLine, CordeModel::Config& Config);
	
	virtual ~cordeCollisionObject();
	
	void predictMotion(btScalar dt) { } // Will likely eventually call cordeModels final (post collision) update step
	
	void integrateMotion() { } // Will likely eventually call cordeModels final (post collision) update step
	
	void solveConstraints() { } // Might need to pass in dt eventually. Internal dynamics of collision object (internal "constraints")
	
	void defaultCollisionHandler(cordeCollisionObject* otherSoftBody) { }
	
	void defaultCollisionHandler(const btCollisionObjectWrapper* collisionObjectWrap ) { }
	
	

	//
	// Set the solver that handles this soft body
	// Should not be allowed to get out of sync with reality
	// Currently called internally on addition to the world
	void setSolver( cordeSolver *softBodySolver )
	{
		m_softBodySolver = softBodySolver;
	}

	//
	// Return the solver that handles this soft body
	// 
	cordeSolver* getSoftBodySolver()
	{
		return m_softBodySolver;
	}

	//
	// Return the solver that handles this soft body
	// 
	cordeSolver* getSoftBodySolver() const
	{
		return m_softBodySolver;
	}
	
	/// @todo look into tgCast for this behavior
	//
	// Cast
	//

	static const cordeCollisionObject*	upcast(const btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_USER_TYPE)
			return (const cordeCollisionObject*)colObj;
		return 0;
	}
	static cordeCollisionObject*			upcast(btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_USER_TYPE)
			return (cordeCollisionObject*)colObj;
		return 0;
	}
	
private:
	
	/**
	 * The solver that handles this softbody. 
	 */
	cordeSolver* m_softBodySolver;

};
 
 
#endif // CORDE_MODEL
