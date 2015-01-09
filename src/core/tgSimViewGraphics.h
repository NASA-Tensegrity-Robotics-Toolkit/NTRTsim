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

#ifndef TG_SIM_VIEW_GRAPHICS_H
#define TG_SIM_VIEW_GRAPHICS_H

/**
 * @file tgSimViewGraphics.h
 * @brief Contains the definition of class tgSimViewGraphics
 * @author Brian Mirletz, Ryan Adams
 * $Id$
 */

// This application
#include "tgSimView.h"
#include "tgBulletRenderer.h"
// Bullet OpenGL_FreeGlut (patched files)
#include "tgGlutStuff.h"
// The Bullet Physics library
#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "tgGlutDemoApplication.h"
#define PlatformDemoApplication tgGlutDemoApplication
#endif

#include "LinearMath/btAlignedObjectArray.h"
// The C++ Standard library
#include <iostream>

// Forward declarations
class tgGLDebugDrawer;


// @todo: Provide ability to make render rate and simulation step rate independent

class tgSimViewGraphics :  public tgSimView, public PlatformDemoApplication
{
public:

    /**
     * The only constructor.
     * @param[in] world a reference to the tgWorld being simulated.
     * @param[in] stepSize the time interval for advancing the simulation;
     * std::invalid_argument is thrown if not positive
     * @param[in] renderRate the time interval for updating the graphics;
     * std::invalid_argument is thrown if less than stepSize
     * @throw std::invalid_argument if stepSize is not positive or renderRate is
     * less than stepSize
     */
    tgSimViewGraphics(tgWorld& world,
              double stepSize = 1.0/120.0,
              double renderRate = 1.0/60.0);
    
    //Exit physics should have already been called
        //exitPhysics();
    virtual ~tgSimViewGraphics();
    
    /**
     * Run until user stops - currently the same as run(steps) due
     * to use of GLUT
     */    
    void run()
    {
        run(10);
    }
    
    /**
     * Gives us a pointer to the world, sets up the debug drawer for
     * the dynamics world and sets up a new tgBulletRenderer
     */
    void setup();
    
    /**
     * Nullifies the pointer to the dynamics world, calls tgSimView::teardown()
     */
    void teardown();
    
    /**
     * Clears the openGL buffer and dispatches a tgBulletRenderer to the world
     */
    void render();
    
    /**
     * Run for a specified number of steps. Currently performs the
     * role of run() at this level
     * @todo doesn't work until FreeGLUT or another rendering engine
     * is implemented
     */
    virtual void run(int steps);
    
    /**
     * Resets the simulation using simulation->reset()
     * the simulation will call setup and teardown on this as appropreate
     */
    void reset();

    //Required by tgDemoApplication
    void    initPhysics(){
        //Hope we've been setup by the time this is called
        if(!isInitialzed()){
            std::cerr << "Attempted initPhysics without setup!" << std::endl;
        }
    }

    //Required by tgDemoApplication
    void exitPhysics(){
        std::cout << "exiting physics" << std::endl;
        teardown();
    }
    
    /**
     * Requried by tgDemoApplication. Handles Rendering
     */
    virtual void clientMoveAndDisplay();
    
    /**
     * Contains a smaller set of rendering functions than clientMoveAndDisplay
     */
    virtual void displayCallback();
    
    /**
     * Called when the space bar is pressed. Calls reset,
     * then updates the pointers to the world and glDebugDrawer
     */
    virtual void clientResetScene();

private:    
    tgGLDebugDrawer*    gDebugDrawer;   
};



#endif
