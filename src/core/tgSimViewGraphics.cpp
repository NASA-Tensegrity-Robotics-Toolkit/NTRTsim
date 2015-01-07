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
// Bullet OpenGL_FreeGlut (patched files)
#include "tgGLDebugDrawer.h"
// The Bullet Physics library
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

tgSimViewGraphics::tgSimViewGraphics(tgWorld& world,
                     double stepSize,
                     double renderRate) : 
  tgSimView(world, stepSize, renderRate)
{
    /// @todo figure out a good time to delete this
    gDebugDrawer = new tgGLDebugDrawer();
    // Supress compiler warning for bullet's unused variable
    (void) btInfinityMask;
}

tgSimViewGraphics::~tgSimViewGraphics()
{
#ifndef BT_NO_PROFILE
    CProfileManager::Release_Iterator(m_profileIterator);
#endif //BT_NO_PROFILE
    delete m_shootBoxShape;
    delete m_shapeDrawer;
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

        // Give the pointer to demoapplication for rendering
        dynamicsWorld.setDebugDrawer(gDebugDrawer);
        
        // @todo Valgrind thinks this is a leak. Perhaps its a GLUT issue?
        m_pModelVisitor = new tgBulletRenderer(world);
        std::cout << "setup graphics" << std::endl;
}

void tgSimViewGraphics::teardown()
{
    //tgWorld owns this pointer, so we shouldn't delete it
    m_dynamicsWorld = 0;
    tgSimView::teardown();
}

void tgSimViewGraphics::render()
{
    if (m_pSimulation && m_pModelVisitor)
    {
        
        glClear(GL_COLOR_BUFFER_BIT |
            GL_DEPTH_BUFFER_BIT |
            GL_STENCIL_BUFFER_BIT);
        
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
        tgglutmain(1024, 600, "Tensegrity Demo", this);

        glutMainLoop();
        
        /* Free glut code
        // This would normally run forever, but this is just for testing
        for(int i = 0; i < steps; i++) {
            m_pSimulation->step(dt);
            
            if(i % 5 == 0) {
                render();
            }
            if(getExitFlag())
            {
                break;
            }   
        }
        std::cout << "leaving loop" << std::endl;
        glutLeaveMainLoop();
        */
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

void tgSimViewGraphics::clientMoveAndDisplay()
{
    if (isInitialzed()){
        m_pSimulation->step(m_stepSize);    
        m_renderTime += m_stepSize; 
        if (m_renderTime >= m_renderRate)
        {
            render();
            // Doesn't appear to do anything yet...
            m_dynamicsWorld->debugDrawWorld();
            renderme();     
            // Camera is updated in renderme
            glFlush();
            swapBuffers();      
            m_renderTime = 0;
        }
    }
}

void tgSimViewGraphics::displayCallback()
{
    if (isInitialzed())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
        renderme();
        // optional but useful: debug drawing to detect problems
        if (m_dynamicsWorld)
        {
            m_dynamicsWorld->debugDrawWorld();
        }
        glFlush();
        swapBuffers();
    }
}

void tgSimViewGraphics::clientResetScene()
{
    reset();
    assert(isInitialzed());

    tgWorld& world = m_pSimulation->getWorld();
    tgBulletUtil::worldToDynamicsWorld(world).setDebugDrawer(gDebugDrawer);
}
