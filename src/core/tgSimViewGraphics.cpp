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
   @author Comments Author: Alex Popescu
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
        // Create a new tgBulletRenderer with a world and shape drawer.
        m_pModelVisitor = new tgBulletRenderer(world, m_shapeDrawer, m_sundirection);
        
        // The shapedrawer is set up in the tgDemoApplication superclass.
        m_shapeDrawer->enableTexture(false);
	    m_enableshadows = true;
        std::cout << "Setup graphics." << std::endl;
}

void tgSimViewGraphics::teardown()
{
    //tgWorld owns this pointer, so we shouldn't delete it
    m_dynamicsWorld = 0;
    tgSimView::teardown();
}

/* This function is called each display_update timestep, to render the scene elements
   that NTRT renders.
*/
void tgSimViewGraphics::render()
{
    //std::cout << "tgSimViewGraphics::render()" << std::endl;
    if (m_pSimulation && m_pModelVisitor)
    {
        /* Clears the color, depth, stencil buffers, giving us a "blank slate" of the
           of the background color for us (and others) to draw upon. If you don't do this,
           objects from last frame will appear on the new frame! */
        glClear(GL_COLOR_BUFFER_BIT |
            GL_DEPTH_BUFFER_BIT |
            GL_STENCIL_BUFFER_BIT);
            
        // Set lighting information:
        GLfloat light_ambient[] = { btScalar(0.2), btScalar(0.2), btScalar(0.2), btScalar(1.0) };
	    GLfloat light_diffuse[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0) };
	    GLfloat light_specular[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0 )};
	    /*	light_position is NOT default value	*/
	    GLfloat light_position0[] = { btScalar(1.0), btScalar(10.0), btScalar(1.0), btScalar(0.0 )};
	    GLfloat light_position1[] = { btScalar(-1.0), btScalar(-10.0), btScalar(-1.0), btScalar(0.0) };

	    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	    glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

	    glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
	    glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

	    glEnable(GL_LIGHTING);
	    glEnable(GL_LIGHT0);
	    glEnable(GL_LIGHT1);

        // Set shading information:
	    glShadeModel(GL_SMOOTH);
	    glEnable(GL_DEPTH_TEST);
	    glDepthFunc(GL_LESS);
        
        // Set clear color (background color).
	    glClearColor(btScalar(1),btScalar(1),btScalar(1),btScalar(0));

	    //  glEnable(GL_CULL_FACE);
	    //  glCullFace(GL_BACK);
	
	    // Update camera:
	    const btVector3 ps = tgDemoApplication::getPs();

	    if (m_autocam)
	    {
		    m_cameraTargetPosition += (ps - m_cameraTargetPosition) * 0.05;
	    }
	    updateCamera();

	    if (m_dynamicsWorld)
	    {			
		    if(m_enableshadows)
		    {
			    glClear(GL_STENCIL_BUFFER_BIT);
			    glEnable(GL_CULL_FACE);
			    renderscene(0);

			    glDisable(GL_LIGHTING);
			    glDepthMask(GL_FALSE);
			    glDepthFunc(GL_LEQUAL);
			    glEnable(GL_STENCIL_TEST);
			    glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
			    glStencilFunc(GL_ALWAYS,1,0xFFFFFFFFL);
			    glFrontFace(GL_CCW);
			    glStencilOp(GL_KEEP,GL_KEEP,GL_INCR);
			    renderscene(1);
			    glFrontFace(GL_CW);
			    glStencilOp(GL_KEEP,GL_KEEP,GL_DECR);
			    renderscene(1);
			    glFrontFace(GL_CCW);

			    glPolygonMode(GL_FRONT,GL_FILL);
			    glPolygonMode(GL_BACK,GL_FILL);
			    glShadeModel(GL_SMOOTH);
			    glEnable(GL_DEPTH_TEST);
			    glDepthFunc(GL_LESS);
			    glEnable(GL_LIGHTING);
			    glDepthMask(GL_TRUE);
			    glCullFace(GL_BACK);
			    glFrontFace(GL_CCW);
			    glEnable(GL_CULL_FACE);
			    glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);

			    glDepthFunc(GL_LEQUAL);
			    glStencilFunc( GL_NOTEQUAL, 0, 0xFFFFFFFFL );
			    glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP );
			    glDisable(GL_LIGHTING);
			    renderscene(2);
			    glEnable(GL_LIGHTING);
			    glDepthFunc(GL_LESS);
			    glDisable(GL_STENCIL_TEST);
			    glDisable(GL_CULL_FACE);
		    }
		    else
		    {
			    glDisable(GL_CULL_FACE);
			    renderscene(0);
		    }

		    int	xOffset = 10;
		    int yStart = 20;
		    int yIncr = 20;


		    glDisable(GL_LIGHTING);
		    glColor3f(0, 0, 0);
            /*
            // Show Bullet profiler
		    if ((m_debugMode & btIDebugDraw::DBG_NoHelpText)==0)
		    {
			    setOrthographicProjection();

			    showProfileInfo(xOffset,yStart,yIncr);

			    resetPerspectiveProjection();
		    }
            */
		    glDisable(GL_LIGHTING);
	    }
	    updateCamera();
    }
}

/* Renders the scene with a certain pass value */
void tgSimViewGraphics::renderscene(int pass)
{
    /* Calls tgSimulation::onVisit(.), which will call the onVisit(.) method of each 
       model and its children, which calls tgBulletRenderer::render(obj). This implements
       our custom rendering methods for each object. */

    // Set pass & debug modes:
    m_pModelVisitor->setPass(pass);
    m_pModelVisitor->setDebugMode(getDebugMode());
    // Draw models:
    m_pSimulation->onVisit(*m_pModelVisitor);
    // Draw ground:
    tgWorld& world = m_pSimulation->getWorld();
    const tgGround& ground = world.getGround();
    const tgBulletGround* bulletGround = dynamic_cast<const tgBulletGround*>(&ground);
    if (bulletGround != NULL)
        m_pModelVisitor->render(*bulletGround); // Render ground if it is a bulletGround.
}

/* This function is called by our custom application to bring up the display window,
   and start the simulation. */
void tgSimViewGraphics::run(int steps) 
{
    if (isInitialzed())
    {
        /* This initializes GLUT and creates a display window (running in a new thread).
           Then, OpenGL shading and light information are set, as are keyboard callback
           functions. Importantly, we set the glutIdleFunc, glutDisplayFunc to functions
           in the tgDemoApplication class, but since this (tgSimViewGraphics) is a derived
           class of that base class, we implement some of the tgSimViewGraphics methods:
               clientMoveAndDisplay()
               displayCallback()
               clientResetScene()
           Finally, clientMoveAndDisplay is called once.
           Also, note the tgDemoApplication is heavily modified from its default state
           through a patch that is applied when NTRT is setup, see
           bin/setup/setup_bullet.sh.
        */
        tgglutmain(1024, 600, "Tensegrity Demo", this); // Specify window size and title.
        
        /* Enter the FreeGLUT main loop. This function will call glutMainLoopEvent() when
           necessary and sleep or call glutIdleFunc() when no render is necessary. 
           We set the following FreeGLUT callbacks in tgglutmain, above:
               glutIdleFunc: called when FreeGLUT is done rendering
                 \-> tgDemoApplication::moveAndDisplay() -> tgSimViewGraphics::clientMoveAndDisplay() if not m_idle,
                        or tgSimViewGraphics::displayCallback(), if m_idle. m_idle is a special idle flag set in
                        the demo application. The default is m_idle=false.
               glutDisplayFunc: OpenGLUT will call whenever it thinks that the window may
                 |              require updating.
                 \-> tgDemoApplication::displayCallback() -> tgSimViewGraphics::displayCallback
        */
        glutMainLoop();
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

/* This function is the idle function called by the FreeFLUT event loop, with the default m_idle value
   (m_idle=false). That means this function is called whenever the window doesn't need to be rendered, making
   it a good place to run the physics simulation. This function is called *continuously* when rendering isn't
   taking place.
   It steps the Bullet simulation 1 time step (physics timestep), and checks if the display timestep has been
   reached, to force a display update at that time */
void tgSimViewGraphics::clientMoveAndDisplay()
{
    //std::cout << "Physics Step" << std::endl;
    static double time = 0;
    
    if (isInitialzed()){
        /* Step the tgSimulation. This calls step() on the Bullet btDiscreteDynamicsWorld, and on each model,
           child of the models, observers, and data manager. Using the physics timestep (small). */
        m_pSimulation->step(m_stepSize);    
        m_renderTime += m_stepSize; // Step the time since last render
        time += m_stepSize;
        if (m_renderTime >= m_renderRate)
        {
            //std::cout << "Worldtime: " << time << std::endl;
            // The time since last render has reached the render rate, so force a render. 
            displayCallback(); // Render
            m_renderTime = 0; // Reset time since last render.
        }
    }
}

/* OpenGLUT will call this whenever it thinks that the window may require updating. This is considered
   as 'non-idle' time, and the OpenGLUT main event loop will complete this before going back to 'idling'
   by doing physics updates. For example, this will call when window is resized.*/
void tgSimViewGraphics::displayCallback()
{
    if (isInitialzed())
    {
        /* This is tgSimViewGraphics::render() implementation, which visits each rigid body and renders it
           to a rendering buffer. */
        render();
        
        // Call debugDrawWorld(), which draws body coordinate frames and wireframes in wireframe mode ONLY.
        m_dynamicsWorld->debugDrawWorld();
        
        // Empty all graphics buffers.
        glFlush();
        
        /* Calls glutSwapBuffers(). Note, due to vsync, the buffers are swapped at max 60Hz, so this
           call will limit the performance of our simulation (as long as we are displaying graphics).
           By removing this call, we can let Bullet physics run as fast as possible. */
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
