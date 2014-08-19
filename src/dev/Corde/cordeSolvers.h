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

This class is a modified version of btSoftBodySolvers
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

#ifndef CORDE_SOLVERS_H
#define CORDE_SOLVERS_H

#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"

class btVertexBufferDescriptor;
class btCollisionObject;
class cordeCollisionObject;


class cordeSolver
{
public:
	enum SolverTypes
	{
		DEFAULT_SOLVER
	};


protected:
	// Simulation timescale
	float m_timeScale;
	
public:
	cordeSolver() :
		m_timeScale( 1 )
	{

	}

	virtual ~cordeSolver()
	{
	}
	
	/**
	 * Return the type of the solver.
	 */
	virtual SolverTypes getSolverType() const = 0;


	/** Ensure that this solver is initialized. */
	virtual bool checkInitialized() = 0;

	/** Optimize soft bodies in this solver. */
	virtual void optimize( btAlignedObjectArray< cordeCollisionObject * > &softBodies , bool forceUpdate=false) = 0;

	/** Copy necessary data back to the original soft body source objects. */
	virtual void copyBackToSoftBodies(bool bMove = true) = 0;

	/** Predict motion of soft bodies into next timestep */
	virtual void predictMotion( float solverdt ) = 0;

	/** Solve constraints for a set of soft bodies */
	virtual void solveConstraints( float solverdt ) = 0;

	/** Perform necessary per-step updates of soft bodies such as recomputing normals and bounding boxes */
	virtual void updateSoftBodies() = 0;

	/** Process a collision between one of the world's soft bodies and another collision object */
	virtual void processCollision( cordeCollisionObject *, const struct btCollisionObjectWrapper* ) = 0;

	/** Process a collision between two soft bodies */
	virtual void processCollision( cordeCollisionObject*, cordeCollisionObject* ) = 0;

	/** Return the timescale that the simulation is using */
	float getTimeScale()
	{
		return m_timeScale;
	}

};

/** 
 * Class to manage movement of data from a solver to a given target.
 * This version is abstract. Subclasses will have custom pairings for different combinations.
 */
class btSoftBodySolverOutput
{
protected:

public:
	btSoftBodySolverOutput()
	{
	}

	virtual ~btSoftBodySolverOutput()
	{
	}


	/** Output current computed vertex data to the vertex buffers for all cloths in the solver. */
	virtual void copySoftBodyToVertexBuffer( const cordeCollisionObject * const cordeCollisionObject, btVertexBufferDescriptor *vertexBuffer ) = 0;
};


#endif // #ifndef CORDE_SOLVERS_H
