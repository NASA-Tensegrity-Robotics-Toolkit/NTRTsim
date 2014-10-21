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
#include "Muscle2P.h"
#include "MuscleAnchor.h"
#include "tgBulletUtil.h"
#include "tgLinearString.h"
#include "tgWorld.h"
#include "tgWorldBulletPhysicsImpl.h"
// OpenGL_FreeGlut (patched Bullet)
#include "tgGLDebugDrawer.h"
// The Bullet Physics library
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
// The C++ Standard Library
#include <cassert>


tgBulletRenderer::tgBulletRenderer(tgWorld& world) : m_world(world)
{
}

void tgBulletRenderer::render(const tgRod& rod) const
{
        // render the rod (change color, etc. if we want)
}

void tgBulletRenderer::render(const tgLinearString& linString) const
{
        // Fetch the btDynamicsWorld
        btDynamicsWorld& dynamicsWorld =
      tgBulletUtil::worldToDynamicsWorld(m_world);

    btIDebugDraw* const pDrawer = dynamicsWorld.getDebugDrawer();
    
    const Muscle2P* const pMuscle = linString.getMuscle();
    
    if (pDrawer && pMuscle)
    {
    
      const btVector3 lineFrom =
        pMuscle->anchor1->getWorldPosition();
      const btVector3 lineTo = 
        pMuscle->anchor2->getWorldPosition();
       // Should this be normalized??
      const double stretch = 
        linString.getCurrentLength() - pMuscle->getRestLength();
      const btVector3 color =
        (stretch < 0.0) ?
        btVector3(0.0, 0.0, 1.0) :
        btVector3(0.5 + stretch / 3.0, 
              0.5 - stretch / 2.0, 
              0.0);
      pDrawer->drawLine(lineFrom, lineTo, color);
    }
}

void tgBulletRenderer::render(const tgModel& model) const
{

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

