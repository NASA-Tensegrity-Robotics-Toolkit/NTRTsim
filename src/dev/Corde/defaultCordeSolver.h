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

This class is a modified version of btDefaultSoftBodySolver
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

#ifndef DEFAULT_CORDE_SOLVER_H
#define DEFAULT_CORDE_SOLVER_H


#include "cordeSolver.h"

/// @todo evaluate necessity of vertex buffer, etc.
#include "btSoftBodySolverVertexBuffer.h"
struct btCollisionObjectWrapper;

class cordeDefaultSolver : public cordeSolver
{
protected:		
	/** Variable to define whether we need to update solver constants on the next iteration */
	bool m_updateSolverConstants;

	btAlignedObjectArray< cordeCollisionObject * > m_softBodySet;


public:
	cordeDefaultSolver();
	
	virtual ~cordeDefaultSolver();
	
	virtual SolverTypes getSolverType() const
	{
		return DEFAULT_SOLVER;
	}

	virtual bool checkInitialized();

	virtual void updateSoftBodies( );

	virtual void optimize( btAlignedObjectArray< cordeCollisionObject * > &softBodies,bool forceUpdate=false );

	virtual void copyBackToSoftBodies(bool bMove = true);

	virtual void solveConstraints( float solverdt );

	virtual void predictMotion( float solverdt );

	virtual void copySoftBodyToVertexBuffer( const cordeCollisionObject *const cordeCollisionObject, btVertexBufferDescriptor *vertexBuffer );

	virtual void processCollision( cordeCollisionObject *, const btCollisionObjectWrapper* );

	virtual void processCollision( cordeCollisionObject*, cordeCollisionObject* );

};

#endif // #ifndef DEFAULT_CORDE_SOLVER_H
