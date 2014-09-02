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
 * 
This class is a modified version of btSoftBody
* from Bullet 2.82. A copy of the z-lib license from the Bullet Physics
* Library is provided below:

Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
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
#include "BulletSoftBody/btSparseSDF.h"

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
class tgWorld;

class cordeCollisionObject : public CordeModel, public btCollisionObject
{
	/**
	 * cordeCollisionShape needs access to bounds, etc, but that's the only class that needs them
	 */
	friend class cordeCollisionShape;
	
	/**
	 * cordeColliders needs access to solver state
	 */
	friend struct cordeColliders;
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
	/* sCti is Softbody contact info	*/ 
	struct	sCti
	{
		const btCollisionObject*	m_colObj;		/* Rigid body			*/ 
		btVector3		m_normal;	/* Outward normal		*/ 
		btScalar		m_offset;	/* Offset from origin	*/ 
		btScalar		m_dist; 	/* Interpenitration distance? */
	};		
	/* RContact		*/ 
	struct	RContact
	{
		sCti		m_cti;			// Contact infos
		CordePositionElement*	m_node;			// Owner node
		btMatrix3x3				m_c0;			// Impulse matrix (8/28/14 - unused)
		btVector3				m_c1;			// Relative anchor (contact point)
		btScalar				m_c2;			// Mass ratio
		btVector3				m_c3;			// Friction Vector and coefficient (@todo, consider seperating)
		/// Removed Contact Hardness
	};
	/* SContact		*/ 
	struct	SContact
	{
		CordePositionElement*	m_node;			// Node
		btVector3				m_weights;		// Weigths
		btVector3				m_normal;		// Normal
		btScalar				m_margin;		// Margin
		btScalar				m_friction;		// Friction
		btScalar				m_cfm[2];		// Constraint force mixing
	};
	
public:
	// Used in append anchors, unused otherwise
	btAlignedObjectArray<const class btCollisionObject*> m_collisionDisabledObjects;

	cordeCollisionObject(std::vector<btVector3>& centerLine, tgWorld& world, CordeModel::Config& Config);
	
	virtual ~cordeCollisionObject();
	
	void predictMotion(btScalar dt); // Will likely eventually call cordeModels final (post collision) update step
	
	// Final part of step loop
	void integrateMotion(btScalar dt);
	
	// Constrained motion - anchors, apply contact forces (8/28 just rigid)
	void solveConstraints();
	
	void defaultCollisionHandler(cordeCollisionObject* otherSoftBody) { }
	
	// Just adds contact points etc, still need to process collisions elsewhere
	void defaultCollisionHandler(const btCollisionObjectWrapper* pcoWrap);
	
	bool	checkContact(	const btCollisionObjectWrapper* colObjWrap,
									 const btVector3& x,
									 btScalar margin,
									 cordeCollisionObject::sCti& cti);

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

public:
	typedef btAlignedObjectArray<RContact>		tRContactArray;
	typedef btAlignedObjectArray<SContact>		tSContactArray;

	tRContactArray			m_rcontacts;	// Rigid contacts
	tSContactArray			m_scontacts;	// Soft contacts
private:
	
	/**
	 * Update the collision bounds of the AABB
	 */
	void updateAABBBounds();
	
	void solveRContacts();
	
	/**
	 * The solver that handles this softbody. 
	 */
	cordeSolver* m_softBodySolver;
	
	/**
	 * The broadphase from the tgWorld (bullet impl)
	 */
	btBroadphaseInterface* m_broadphase;
	btDispatcher* m_dispatcher;
	/// If you store this elsewhere, checkContacts becomes const
	btSparseSdf<3>			m_sparsesdf;
	/**
	 * Collision Data
	 */
	 /// @todo consider adding struct info about whether or not there is an anchor
	SolverState					m_sst;			// Solver state
	
	/**
	 * @todo look at how this should be cleared out. Delete causes double free error
	 */
	std::vector<btDbvtNode*> 	m_leaves;		// Leaves, should have length same as m_massPoints
	btDbvt						m_ndbvt;		// Nodes tree
	btVector3					m_bounds[2];	// Spatial bounds
};
 
 
#endif // CORDE_MODEL
