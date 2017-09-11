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

/**
 * @file tgBulletRenderer.cpp
 * @brief Contains the definitions of members of class tgBulletRenderer
 * @author Brian Tietz, Alex Popescu
 * $Id$
 */

// This module
#include "tgBulletRenderer.h"
// This application
#include "abstractMarker.h"
#include "tgSpringCable.h"
#include "tgBulletCompressionSpring.h"
#include "tgSpringCableAnchor.h"
#include "tgBulletUtil.h"
#include "tgSpringCableActuator.h"
#include "tgCompressionSpringActuator.h"
#include "tgWorld.h"
#include "tgWorldBulletPhysicsImpl.h"
#include "tgGLDebugDrawer.h"
#include "tgGlutDemoApplication.h"

#include "tgCast.h"

#include "LinearMath/btQuickprof.h"

// OpenGL_FreeGlut (patched Bullet)
#include "tgGLDebugDrawer.h"
// The Bullet Physics library
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btUniformScalingShape.h"
#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"
#include "LinearMath/btDefaultMotionState.h"
#include "GL_ShapeDrawer.h"
// The C++ Standard Library
#include <cassert>


// This requires a reference to a shape drawer and tgWorld to be constructed.
tgBulletRenderer::tgBulletRenderer(tgWorld& world, GL_ShapeDrawer* shapeDrawer, btVector3 sunAngle)
    : m_world(world),
      m_shapeDrawer(shapeDrawer),
      m_sunAngle(sunAngle)
{
}

/* Custom rendering function for rigid bodies */
void tgBulletRenderer::render(const tgBaseRigid& rigid) const
{
    //std::cout << "Render rigid body" <<std::endl;
    if(getDebugMode()& btIDebugDraw::DBG_DrawWireframe) return; // Do not render in wireframe mode!
    // Save profiler information, to see how long rendering takes:
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgBulletRenderer::render(rigid)");
#endif //BT_NO_PROFILE 
    btDynamicsWorld& dynamicsWorld = tgBulletUtil::worldToDynamicsWorld(m_world);
    btVector3 aabbMin,aabbMax;
	dynamicsWorld.getBroadphase()->getBroadphaseAabb(aabbMin,aabbMax);
	aabbMin-=btVector3(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	aabbMax+=btVector3(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	
	btScalar m[16];
	btMatrix3x3	rot;
	rot.setIdentity();
	
	// Get the collision object:
	const btRigidBody* body = rigid.getPRigidBody();
	// Get the display shape:
	const btCollisionShape* displayShape = rigid.getDisplayShape();
	
	// Get the transform matrix for the rigid body:
	if(body&&body->getMotionState())
	{
		btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
		myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
		rot=myMotionState->m_graphicsWorldTrans.getBasis();
	}
	else
	{
		body->getWorldTransform().getOpenGLMatrix(m);
		rot=body->getWorldTransform().getBasis();
	}
	
	// Note: we render the display shape, with the transform of the collision object.
	
	// Set texture and color:
	m_shapeDrawer->enableTexture(rigid.getDrawTextureOn());
	btVector3 shapeColor = rigid.getDrawColor();
	
	// Draw shape:
    switch(getPass())
	{
	case	0:	m_shapeDrawer->drawOpenGL(m,displayShape,shapeColor,0,aabbMin,aabbMax);break;
	case	1:	m_shapeDrawer->drawShadow(m,m_sunAngle*rot,displayShape,aabbMin,aabbMax);break;
	case	2:	m_shapeDrawer->drawOpenGL(m,displayShape,shapeColor*btScalar(0.5),0,aabbMin,aabbMax);break;
	}
}

/* Custom rendering function for ground */
void tgBulletRenderer::render(const tgBulletGround& ground) const
{
    //std::cout << "Render ground" <<std::endl;
    // Save profiler information, to see how long rendering takes:
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgBulletRenderer::render(ground)");
#endif //BT_NO_PROFILE 
    if(getDebugMode()& btIDebugDraw::DBG_DrawWireframe) return; // Do not render in wireframe mode!
    btDynamicsWorld& dynamicsWorld = tgBulletUtil::worldToDynamicsWorld(m_world);
    btVector3 aabbMin,aabbMax;
	dynamicsWorld.getBroadphase()->getBroadphaseAabb(aabbMin,aabbMax);
	aabbMin-=btVector3(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	aabbMax+=btVector3(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	
	btScalar m[16];
	btMatrix3x3	rot;
	rot.setIdentity();
	
	// Get the collision object:
	const btRigidBody* body = ground.getGroundRigidBody();
	const btCollisionShape* colShape = body->getCollisionShape();
	
	// Get the transform matrix for the body:
	if(body&&body->getMotionState())
	{
		btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
		myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
		rot=myMotionState->m_graphicsWorldTrans.getBasis();
	}
	else
	{
		body->getWorldTransform().getOpenGLMatrix(m);
		rot=body->getWorldTransform().getBasis();
	}
	
	btVector3 shapeColor(1, 1, 1); // Ground color
	
	m_shapeDrawer->enableTexture(true); // Ground texture
	
	// Draw shape:
    switch(getPass())
	{
	case	0:	m_shapeDrawer->drawOpenGL(m,colShape,shapeColor,0,aabbMin,aabbMax);break;
	case	1:	m_shapeDrawer->drawShadow(m,m_sunAngle*rot,colShape,aabbMin,aabbMax);break;
	case	2:	m_shapeDrawer->drawOpenGL(m,colShape,shapeColor*btScalar(0.5),0,aabbMin,aabbMax);break;
	}
}

void tgBulletRenderer::render(const tgSpringCableActuator& mSCA) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgBulletRenderer::renderString");
#endif //BT_NO_PROFILE 
    //std::cout<<"Draw string"<<std::endl;
        // Fetch the btDynamicsWorld
        btDynamicsWorld& dynamicsWorld =
      tgBulletUtil::worldToDynamicsWorld(m_world);
    
    // Disable texture for drawing lines:
    glDisable (GL_TEXTURE_2D);
    
    btIDebugDraw* const pDrawer = dynamicsWorld.getDebugDrawer();
    
    const tgSpringCable* const pSpringCable = mSCA.getSpringCable();
    
    if(pDrawer && pSpringCable)
    {
		const std::vector<const tgSpringCableAnchor*>& anchors = pSpringCable->getAnchors();
		std::size_t n = anchors.size() - 1;
		for (std::size_t i = 0; i < n; i++)
		{
			const btVector3 lineFrom =
			anchors[i]->getWorldPosition();
		  const btVector3 lineTo = 
			anchors[i+1]->getWorldPosition();
		   // Should this be normalized??
		  const double stretch = 
			mSCA.getCurrentLength() - mSCA.getRestLength();
		  const btVector3 color =
			(stretch < 0.0) ?
			btVector3(0.0, 0.0, 1.0) :
			btVector3(0.5 + stretch / 3.0, 
				  0.5 - stretch / 2.0, 
				  0.0);
		  pDrawer->drawLine(lineFrom, lineTo, color);
		}
	}
}

/**
 * Render a tgCompressionSpringActuator
 * Just like a spring-cable actuator.
 */
void tgBulletRenderer::render(const tgCompressionSpringActuator& mCSA) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgBulletRenderer::renderCompressionSpring");
#endif //BT_NO_PROFILE 
    // Fetch the btDynamicsWorld
    btDynamicsWorld& dynamicsWorld = tgBulletUtil::worldToDynamicsWorld(m_world);

    btIDebugDraw* const pDrawer = dynamicsWorld.getDebugDrawer();
    
    const tgBulletCompressionSpring* const pCompressionSpring =
      mCSA.getCompressionSpring();
    
    if(pDrawer && pCompressionSpring)
    {
		const std::vector<const tgSpringCableAnchor*>& anchors =
		  pCompressionSpring->getAnchors();
		// This for loop is designed for the multiple-anchor SpringCable,
		// but works fine here too where there are only two anchors.
		std::size_t n = anchors.size() - 1;
		for (std::size_t i = 0; i < n; i++)
		{

		  // This method assumes the spring is "attached" to the first
		  // anchor, although it doesn't really matter mathematically.
		  const btVector3 springStartLoc =
			 anchors[i]->getWorldPosition();

		  // Have the spring state its end point location, instead of
		  // assuming it's at anchor 2.
		  const btVector3 springEndLoc =
		    pCompressionSpring->getSpringEndpoint();

		  // The 'color' variable will be set according to if the
		  // free end of the spring is attached or not.
		  btVector3 color;
		  
		  // Different behavior depending on if the spring is attached
		  // at its free end.
		  // If it *is* attached, render like a SpringCable:

		  if( pCompressionSpring->isFreeEndAttached() )
		  {
		    // The color of the spring is according to its current force.
		    // According to the Bullet Physics API for btIDebugDraw.drawLine,
		    // "For color arguments the X,Y,Z components refer to
		    // Red, Green and Blue each in the range [0..1]"
		    // Let's do red for tension and green for compression
		    // (note that if the free end is attached, there is always an
		    // applied force, no "blue" for no force.")
		    // Negative forces pull the anchors together, positive pushes
		    // them apart.
		    color = 
		      ( pCompressionSpring->getSpringForce() < 0.0 ) ?
		      btVector3(1.0, 0.0, 0.0) :
		      btVector3(0.0, 1.0, 0.0);
		  }
		  else
		  {
		    // Like above, the color will depend on force.
		    // Here, though, there will never be any tension force,
		    // since the spring will provide zero force when anchor distance
		    // is greater than rest length.
		    // Less than or equal to zero: blue, no force
		    color = 
		      ( pCompressionSpring->getSpringForce() <= 0.0 ) ?
		      btVector3(0.0, 0.0, 1.0) :
		      btVector3(0.0, 1.0, 0.0);
		  }
		  // Draw the string, now that color has been set.
		  //pDrawer->drawLine(springStartLoc, springEndLoc, color);
		}
	}
}

void tgBulletRenderer::render(const tgModel& model) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgBulletRenderer::renderModel");
#endif //BT_NO_PROFILE  
	/**
	 * Render the markers of the model using spheres.
	 */

	// Fetch the btDynamicsWorld
	btDynamicsWorld& dynamicsWorld = tgBulletUtil::worldToDynamicsWorld(m_world);
	btIDebugDraw* const idraw = dynamicsWorld.getDebugDrawer();
	for(int j=0;j<model.getMarkers().size() ;j++)
	{
		abstractMarker mark = model.getMarkers()[j];
		idraw->drawSphere(mark.getWorldPosition(),0.6,mark.getColor());
	}
}

