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

#ifndef PRISM_MODEL_H
#define PRISM_MODEL_H

/**
 * @file PrismModel.h
 * @brief Defines a 3 strut 9 string tensegrity model
 * @author Brian Mirletz
 * @version 1.1.0
 * $Id$
 */

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"
// The C++ Standard Library
#include <vector>
#include <iostream>
#include <fstream>

#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "core/tgString.h"
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
 * A class that constructs a three bar tensegrity prism using the tools
 * in tgcreator. This iteration avoids using a controller and instead
 * uses the new (to v1.1) ability to define pretension in a
 * tgBasicActuator's constructor
 */
class PrismModel : public tgSubject<PrismModel>, public tgModel
{
public:     
    /**
     * The only constructor. Configuration parameters are within the
     * .cpp file in this case, not passed in. 
     */
    PrismModel();
    
    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~PrismModel();
    
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


    /**
     * Return a vector of all muscles for the controllers to work with.
     * @return A vector of all of the muscles
     */
    std::vector<tgBasicActuator*>& getAllActuators();

    std::vector<tgRod*>& getAllRods();

    std::vector<btVector3>& getNormVects();

    std::vector<btRigidBody*>& getTank();

    std::vector<abstractMarker> markers;

    btQuaternion rotateToFace(tgStructure& s, int face);

    void rotateNormVectors(btQuaternion rotation);

    void changeRobotState(int state);

    double rodDist;

    // Nodes at the end of each of the rods
    btVector3 node0;
    btVector3 node1;
    btVector3 node2;
    btVector3 node3;
    btVector3 node4;
    btVector3 node5;
    btVector3 node6;
    btVector3 node7;
    btVector3 node8;
    btVector3 node9;
    btVector3 node10;
    btVector3 node11;

    // Edge vectors of all closed triangles
    btVector3 face0Edge0;
    btVector3 face0Edge1;
    btVector3 face0Edge2;

    btVector3 face2Edge0;
    btVector3 face2Edge1;
    btVector3 face2Edge2;

    btVector3 face5Edge0;
    btVector3 face5Edge1;
    btVector3 face5Edge2;

    btVector3 face7Edge0;
    btVector3 face7Edge1;
    btVector3 face7Edge2;

    btVector3 face8Edge0;
    btVector3 face8Edge1;
    btVector3 face8Edge2;

    btVector3 face10Edge0;
    btVector3 face10Edge1;
    btVector3 face10Edge2;

    btVector3 face13Edge0;
    btVector3 face13Edge1;
    btVector3 face13Edge2;

    btVector3 face15Edge0;
    btVector3 face15Edge1;
    btVector3 face15Edge2;
 
    btTransform* thrusterTransform;
    btDynamicsWorld* btWorld;
    GLDebugDrawer* gDebugDraw;
    btHingeConstraint* altitudeHinge;
    btHingeConstraint* yawHinge;

    /*
    double goalYaw;
    double goalAltitude;
    btVector3 goalVector;
    */
    std::vector<btRigidBody*> ThrusterBodies;
    std::vector<btRigidBody*> TankBodies;

    int robotState = 1;


private:
    
    /**
     * A function called during setup that determines the positions of
     * the nodes based on construction parameters. Rewrite this function
     * for your own models
     * @param[in] s: A tgStructure that we're building into
     * @param[in] edge: the X distance of the base points
     * @param[in] width: the Z distance of the base triangle
     * @param[in] height: the Y distance along the axis of the prism
     */
    static void addNodes(tgStructure& s, int pointOffset, double tankToOuterRing);

    static void addRing(tgStructure& s, double radius, int nPts, int& pointOffset);
    static void makeLinks(tgStructure& s, double extRadius, double intRadius, double tankToOuterRing, double tankRadius, int pointOffset1, int& pointOffset2, int baseStartLink, int gimbalStart);
    static void addBottomStructure(tgStructure& s,double extRadius, double payLength,double tankToOuterRing, int& pointOffset);
    static void addThruster(tgStructure& s, int nExt, int& pointOffset, int gimbalStart);
    static void addStrings(tgStructure& s, int baseStartLinks, int beforeRobotPt);
    static void drawThruster();
    /**
     * A function called during setup that creates rods from the
     * relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addRods(tgStructure& s, int pointOffset);
    
    /**
     * A function called during setup that creates muscles (Strings) from
     * the relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addActuators(tgStructure& s, int pointOffset);

    static void addRobot(tgStructure& s, int& offset, double tankToOuterRing);

    static double* returnCOM(tgStructure &s, int firstNode, int numberofNodes);


private:    
    /**
     * A list of all of the spring cable actuators. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    
    std::vector<tgBasicActuator*> allActuators;
    std::vector<btVector3> normalVectors;
    std::vector<btVector3> normalVectors_rotated;
    std::vector<tgRod*> allRods;

    std::ofstream sim_out;
    
    // Normal vectors of all icosahedron faces
    btVector3 face0Norm;
    btVector3 face1Norm;
    btVector3 face2Norm;
    btVector3 face3Norm;
    btVector3 face4Norm;
    btVector3 face5Norm;
    btVector3 face6Norm;
    btVector3 face7Norm;
    btVector3 face8Norm;
    btVector3 face9Norm;
    btVector3 face10Norm;
    btVector3 face11Norm;
    btVector3 face12Norm;
    btVector3 face13Norm;
    btVector3 face14Norm;
    btVector3 face15Norm;
    btVector3 face16Norm;
    btVector3 face17Norm;
    btVector3 face18Norm;
    btVector3 face19Norm;
};

#endif  // Prism_MODEL_H
