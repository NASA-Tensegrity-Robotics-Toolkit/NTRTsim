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

#include "defaultCordeSolver.h"
#include "cordeCollisionObject.h"

#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"

#include "BulletCollision/CollisionShapes/btCapsuleShape.h"

#include "LinearMath/btQuickprof.h"

defaultCordeSolver::defaultCordeSolver()
{
	// Initial we will clearly need to update solver constants
	// For now this is global for the cloths linked with this solver - we should probably make this body specific 
	// for performance in future once we understand more clearly when constants need to be updated
	m_updateSolverConstants = true;
}

defaultCordeSolver::~defaultCordeSolver()
{
}

// In this case the data is already in the soft bodies so there is no need for us to do anything
void defaultCordeSolver::copyBackToSoftBodies(bool bMove)
{

}

void defaultCordeSolver::optimize( btAlignedObjectArray< cordeCollisionObject * > &softBodies , bool forceUpdate)
{
#ifndef BT_NO_PROFILE
	BT_PROFILE("optimize");
#endif //BT_NO_PROFILE
	m_softBodySet.copyFromArray( softBodies );
}

void defaultCordeSolver::updateSoftBodies(float dt )
{
	
	for ( int i=0; i < m_softBodySet.size(); i++)
	{
		cordeCollisionObject*	psb=(cordeCollisionObject*)m_softBodySet[i];
		if (psb->isActive())
		{
			psb->integrateMotion(dt);	
		}
	}
} // updateSoftBodies

bool defaultCordeSolver::checkInitialized()
{
	return true;
}

void defaultCordeSolver::solveConstraints( float solverdt )
{
	// Solve constraints for non-solver softbodies
	for(int i=0; i < m_softBodySet.size(); ++i)
	{
#ifndef BT_NO_PROFILE		
		BT_PROFILE("SolveConstraints");
#endif //BT_NO_PROFILE
		cordeCollisionObject*	psb = static_cast<cordeCollisionObject*>(m_softBodySet[i]);
		if (psb->isActive())
		{
			
			psb->solveConstraints(solverdt );
		}
	}	
} // defaultCordeSolver::solveConstraints

#if (0) /// @todo restore once we have the proper buffers
void defaultCordeSolver::copySoftBodyToVertexBuffer( const cordeCollisionObject *const cordeCollisionObject, btVertexBufferDescriptor *vertexBuffer )
{
	// Currently only support CPU output buffers
	// TODO: check for DX11 buffers. Take all offsets into the same DX11 buffer
	// and use them together on a single kernel call if possible by setting up a
	// per-cloth target buffer array for the copy kernel.

	if( vertexBuffer->getBufferType() == btVertexBufferDescriptor::CPU_BUFFER )
	{
		const btAlignedObjectArray<btSoftBody::Node> &clothVertices( softBody->m_nodes );
		int numVertices = clothVertices.size();

		const btCPUVertexBufferDescriptor *cpuVertexBuffer = static_cast< btCPUVertexBufferDescriptor* >(vertexBuffer);						
		float *basePointer = cpuVertexBuffer->getBasePointer();						

		if( vertexBuffer->hasVertexPositions() )
		{
			const int vertexOffset = cpuVertexBuffer->getVertexOffset();
			const int vertexStride = cpuVertexBuffer->getVertexStride();
			float *vertexPointer = basePointer + vertexOffset;

			for( int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex )
			{
				btVector3 position = clothVertices[vertexIndex].m_x;
				*(vertexPointer + 0) = position.getX();
				*(vertexPointer + 1) = position.getY();
				*(vertexPointer + 2) = position.getZ();
				vertexPointer += vertexStride;
			}
		}
		if( vertexBuffer->hasNormals() )
		{
			const int normalOffset = cpuVertexBuffer->getNormalOffset();
			const int normalStride = cpuVertexBuffer->getNormalStride();
			float *normalPointer = basePointer + normalOffset;

			for( int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex )
			{
				btVector3 normal = clothVertices[vertexIndex].m_n;
				*(normalPointer + 0) = normal.getX();
				*(normalPointer + 1) = normal.getY();
				*(normalPointer + 2) = normal.getZ();
				normalPointer += normalStride;
			}
		}
	}
} // defaultCordeSolver::copySoftBodyToVertexBuffer
#endif

void defaultCordeSolver::processCollision( cordeCollisionObject* softBody, cordeCollisionObject* otherSoftBody)
{
#ifndef BT_NO_PROFILE	
	BT_PROFILE("ProcessSoftSoftCollision");
#endif // BT_NO_PROFILE	
	softBody->defaultCollisionHandler( otherSoftBody);
}

// For the default solver just leave the soft body to do its collision processing
void defaultCordeSolver::processCollision( cordeCollisionObject* softBody, const btCollisionObjectWrapper* collisionObjectWrap )
{
#ifndef BT_NO_PROFILE	
	BT_PROFILE("ProcessSoftHardCollsion");
#endif // BT_NO_PROFILE
	softBody->defaultCollisionHandler( collisionObjectWrap );
} // defaultCordeSolver::processCollision


void defaultCordeSolver::predictMotion( float timeStep )
{
#ifndef BT_NO_PROFILE
	BT_PROFILE("Predict Motion");
#endif // BT_NO_PROFILE	
	for ( int i=0; i < m_softBodySet.size(); ++i)
	{
		cordeCollisionObject*	psb = m_softBodySet[i];

		if (psb->isActive())
		{
			psb->predictMotion(timeStep);		
		}
	}
}

