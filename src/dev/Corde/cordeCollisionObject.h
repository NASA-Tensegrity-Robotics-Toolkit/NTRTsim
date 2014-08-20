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
#include "BulletCollision/BroadphaseCollision/btDbvt.h"

// Bullet Linear Algebra
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btAlignedObjectArray.h"

// The C++ Standard Library
#include <vector>

class cordeSolver;
class cordeCollisionShape;
class btCollisionObjectWrapper;
class btBroadphaseInterface;
class btDispatcher;
class tgWorldBulletPhysicsImpl;

class cordeCollisionObject : public CordeModel, public btCollisionObject
{
public:
	/* SolverState	*/ 
	struct	SolverState
	{
		btScalar				sdt;			// dt*timescale
		btScalar				isdt;			// 1/sdt
		btScalar				velmrg;			// velocity margin
		btScalar				radmrg;			// radial margin
		btScalar				updmrg;			// Update margin
	};	
	
public:

	cordeCollisionObject(std::vector<btVector3>& centerLine, tgWorldBulletPhysicsImpl& world, CordeModel::Config& Config);
	
	virtual ~cordeCollisionObject();
	
	void predictMotion(btScalar dt); // Will likely eventually call cordeModels final (post collision) update step
	
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
	 * Update the collision bounds of the AABB
	 */
	void updateAABBBounds();
	
	/**
	 * The solver that handles this softbody. 
	 */
	cordeSolver* m_softBodySolver;
	
	/**
	 * The broadphase from the tgWorld (bullet impl)
	 */
	btBroadphaseInterface& m_broadphase;
	btDispatcher& m_dispatcher;
	
	/**
	 * Collision Data
	 */
	SolverState					m_sst;			// Solver state
	std::vector<btDbvtNode*> 	m_leaves;		// Leaves, should have length same as m_massPoints
	btDbvt						m_ndbvt;		// Nodes tree
	btVector3					m_bounds[2];	// Spatial bounds
};
 
 
#endif // CORDE_MODEL
