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
 * @author Brian Tietz
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

#include "tgCast.h"

#include "LinearMath/btQuickprof.h"

// OpenGL_FreeGlut (patched Bullet)
#include "tgGLDebugDrawer.h"
// The Bullet Physics library
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
// The C++ Standard Library
#include <cassert>


tgBulletRenderer::tgBulletRenderer(const tgWorld& world) : m_world(world)
{
}

void tgBulletRenderer::render(const tgRod& rod) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgBulletRenderer::renderRod");
#endif //BT_NO_PROFILE 
        // render the rod (change color, etc. if we want)
}

void tgBulletRenderer::render(const tgSpringCableActuator& mSCA) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgBulletRenderer::renderString");
#endif //BT_NO_PROFILE 
        // Fetch the btDynamicsWorld
        btDynamicsWorld& dynamicsWorld =
      tgBulletUtil::worldToDynamicsWorld(m_world);

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
		  pDrawer->drawLine(springStartLoc, springEndLoc, color);
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

