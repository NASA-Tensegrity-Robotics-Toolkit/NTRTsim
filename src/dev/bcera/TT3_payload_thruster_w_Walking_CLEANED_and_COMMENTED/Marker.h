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

#ifndef MARKER_MODEL_H
#define MARKER_MODEL_H

/**
 * @file MarkerModel.h
 * @brief Defines a simple pylon marker 
 * @author Brian Cera
 * @version 1.1.0
 * $Id$
 */

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"
// The C++ Standard Library
#include <vector>

#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
#include "core/tgWorldBulletPhysicsImpl.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
// The C++ Standard Library
#include <stdexcept>
#include <math.h>
#include <vector>

//Debug Drawers
#include "GL_ShapeDrawer.h"
#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"

// Forward declarations
class tgSpringCableActuator;
class tgModelVisitor;
class tgStructure;
class tgWorld;

/**
 * A class that constructs a three bar tensegrity marker using the tools
 * in tgcreator. This iteration avoids using a controller and instead
 * uses the new (to v1.1) ability to define pretension in a
 * tgBasicActuator's constructor
 */
class MarkerModel : public tgSubject<MarkerModel>, public tgModel
{
public:     
    /**
     * The only constructor. Configuration parameters are within the
     * .cpp file in this case, not passed in. 
     */
    MarkerModel(btVector3);
    
    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~MarkerModel();
    
    /**
     * Create the model. Place the rods and strings into the world
     * that is passed into the simulation. This is triggered
     * automatically when the model is added to the simulation, when
     * tgModel::setup(world) is called (if this model is a child),
     * and when reset is called. Also notifies controllers of setup.
     * @param[in] world - the world we're building into
     */
    virtual void setup(tgWorld& world);
    
    /**
     * Undoes setup. Deletes child models. Called automatically on
     * reset and end of simulation. Notifies controllers of teardown
     */
    virtual void teardown();
    
    /**
     * Step the model, its children. Notifies controllers of step.
     * @param[in] dt, the timestep. Must be positive.
     */
    virtual void step(double dt);
    
    /**
     * Receives a tgModelVisitor and dispatches itself into the
     * visitor's "render" function. This model will go to the default
     * tgModel function, which does nothing.
     * @param[in] r - a tgModelVisitor which will pass this model back
     * to itself 
     */
    virtual void onVisit(tgModelVisitor& r);


 

};

#endif  // Marker_MODEL_H
