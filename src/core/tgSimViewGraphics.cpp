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
 * @file tgSimViewGraphics.cpp
 * @brief Contains the definitions of members of class tgSimViewGraphics
 * @author Brian Mirletz, Ryan Adams
 * $Id$
 */

// This module
#include "tgSimViewGraphics.h"
// This application
#include "tgBulletUtil.h"
#include "tgSimulation.h"

// The Bullet Physics library
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "ExampleBrowser/GL_ShapeDrawer.h"

tgSimViewGraphics::tgSimViewGraphics(tgWorld& world,
                     double stepSize,
                     double renderRate) : 
  tgSimView(world, stepSize, renderRate),
  OpenGLExampleBrowser(NULL) // TODO FIX THIS!
{
    /// @todo figure out a good time to delete this
    gDebugDrawer = new GL_ShapeDrawer();
    // Supress compiler warning for bullet's unused variable
    (void) btInfinityMask;
}

tgSimViewGraphics::~tgSimViewGraphics()
{
#ifndef BT_NO_PROFILE
#if 0 // TODO figure out what happend to CProfileManager
    CProfileManager::Release_Iterator(m_profileIterator);
#endif
    #endif //BT_NO_PROFILE

}

void tgSimViewGraphics::setup()
{
        // Just set tgSimView::m_initialized to true
        tgSimView::setup();

        // Cache a pointer to the btSoftRigidDynamicsWorld
        tgWorld& world = m_pSimulation->getWorld();
        btDynamicsWorld& dynamicsWorld =
                tgBulletUtil::worldToDynamicsWorld(world);
        // Store a pointer to the btSoftRigidDynamicsWorld
        // This class is not taking ownership of it
        /// @todo Can this pointer become invalid if a reset occurs?
        m_dynamicsWorld = &dynamicsWorld;

        // @todo Valgrind thinks this is a leak. Perhaps its a GLUT issue?
        m_pModelVisitor = new tgBulletRenderer(world, *gDebugDrawer);
        std::cout << "setup graphics" << std::endl;
}

void tgSimViewGraphics::teardown()
{
    //tgWorld owns this pointer, so we shouldn't delete it
    m_dynamicsWorld = 0;
    tgSimView::teardown();
}

void tgSimViewGraphics::render(double dt)
{
    if (m_pSimulation && m_pModelVisitor)
    {
        
        OpenGLExampleBrowser::update(dt);
        
        m_pSimulation->onVisit(*m_pModelVisitor);

        //Freeglut code
#if (0)
        clientMoveAndDisplay();
        tgGlutMainEventLoop();
#endif
    }
}

void tgSimViewGraphics::run(int steps) 
{
    if (isInitialzed())
    {
        OpenGLExampleBrowser::init(NULL, NULL);
     
        
        // This would normally run forever, but this is just for testing
        for(int i = 0; i < steps; i++) {
            m_pSimulation->step(m_stepSize);
            
            // @todo fix this to actually incorperate m_renderTime
            if(i % 5 == 0) {
                render(m_stepSize * 5);
            }
             
        }

    }
}

// tgSimulation handles calling teardown and setup on this,
// since it knows when the new world is available
void tgSimViewGraphics::reset() 
{
    assert(isInitialzed());
    m_pSimulation->reset();
    assert(isInitialzed());
}
