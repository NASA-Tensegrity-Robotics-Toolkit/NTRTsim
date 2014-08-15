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
 * @file tgWorldBulletPhysicsImpl.cpp
 * @brief Contains the definitions of members of class tgWorldBulletPhysicsImpl
 * @author Lee Brownston
 * $Id$
 */

// This module
#include "tgWorldBulletPhysicsImpl.h"
// This application
#include "tgWorld.h"
#include "terrain/tgBulletGround.h"
#include "terrain/tgHillyGround.h"
// The Bullet Physics library
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btAxisSweep3.h" // New broadphase
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btScalar.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include <iostream> //TODO: Remove after debugging

/**
 * Helper class to bundle objects that have the same life cycle, so they can be
 * constructed and destructed together.
 */
class IntermediateBuildProducts
{
    public:
        IntermediateBuildProducts(double worldSize) : 
            corner1 (-worldSize,-worldSize, -worldSize),
            corner2 (worldSize, worldSize, worldSize),
            dispatcher(&collisionConfiguration),
#if (1) // More acc broadphase - remeber the comma
            broadphase(corner1, corner2, 16384)
#endif
            {
            }
        const btVector3 corner1;
        const btVector3 corner2;
        btSoftBodyRigidBodyCollisionConfiguration collisionConfiguration;
        btCollisionDispatcher dispatcher;
#if (0) // Default broadphase
        btDbvtBroadphase broadphase;
#else
        // More accurate broadphase:
        btAxisSweep3 broadphase;
#endif
        btSequentialImpulseConstraintSolver solver;
};

tgWorldBulletPhysicsImpl::tgWorldBulletPhysicsImpl(const tgWorld::Config& config,
        tgBulletGround* ground) :
    tgWorldImpl(config, ground),
    m_pIntermediateBuildProducts(new IntermediateBuildProducts(config.worldSize)),
    m_pDynamicsWorld(createDynamicsWorld())
{
    // Gravitational acceleration is down on the Y axis
    const btVector3 gravityVector(0, -config.gravity, 0);
    m_pDynamicsWorld->setGravity(gravityVector);
    std::cout << "calling non-Hilly constructor" << std::endl;

    // Create and add the ground rigid body
    #if (1)
        btRigidBody * const pGroundRigidBody = createGroundRigidBody();
        m_pDynamicsWorld->addRigidBody(pGroundRigidBody);
    #else
        m_pDynamicsWorld->addRigidBody(ground->getGroundRigidBody());
    #endif

    #if (1) /// @todo This is a line from the old BasicLearningApp.cpp that we're not using. Investigate further
        m_pDynamicsWorld->getSolverInfo().m_splitImpulse = true;
    #endif	
    // Postcondition
    assert(invariant());
}

tgWorldBulletPhysicsImpl::tgWorldBulletPhysicsImpl(const tgWorld::Config& config,
        tgHillyGround* ground) :
    tgWorldImpl(config, ground),
    m_pIntermediateBuildProducts(new IntermediateBuildProducts(config.worldSize)),
    m_pDynamicsWorld(createDynamicsWorld())
{
    // Gravitational acceleration is down on the Y axis
    const btVector3 gravityVector(0, -config.gravity, 0);
    m_pDynamicsWorld->setGravity(gravityVector);
    std::cout << "calling Hilly constructor" << std::endl;

    // Create and add the ground rigid body
    #if (1)
        btRigidBody * const pGroundRigidBody = createGroundRigidBody();
        m_pDynamicsWorld->addRigidBody(pGroundRigidBody);
    #else
        m_pDynamicsWorld->addRigidBody(ground->getGroundRigidBody());
    #endif

    #if (1) /// @todo This is a line from the old BasicLearningApp.cpp that we're not using. Investigate further
        m_pDynamicsWorld->getSolverInfo().m_splitImpulse = true;
    #endif	
    // Postcondition
    assert(invariant());
}

tgWorldBulletPhysicsImpl::~tgWorldBulletPhysicsImpl()
{
    // Delete all the collision objects. The dynamics world must exist.
    // Delete in reverse order of creation.
    removeConstraints();
    removeCollisionShapes();

    delete m_pDynamicsWorld;

    // Delete the intermediate build products, which are now orphaned
    delete m_pIntermediateBuildProducts;
}

void tgWorldBulletPhysicsImpl::removeCollisionShapes()
{
    // Delete all the collision objects. The dynamics world must exist.
    // Delete in reverse order of creation.
    const size_t nco = m_pDynamicsWorld->getNumCollisionObjects();
    btCollisionObjectArray& oa = m_pDynamicsWorld->getCollisionObjectArray();
    for (int i = nco - 1; i >= 0; --i)
    {
        btCollisionObject * const pCollisionObject = oa[i];

        // If the collision object is a rigid body, delete its motion state
        const btRigidBody* const pRigidBody =
            btRigidBody::upcast(pCollisionObject);
        if (pRigidBody)
        {
            delete pRigidBody->getMotionState();
        }

        // Remove the collision object from the dynamics world
        m_pDynamicsWorld->removeCollisionObject(pCollisionObject);
        // Delete the collision object
        delete pCollisionObject;
    }
    // All collision objects have been removed and deleted
    assert(m_pDynamicsWorld->getNumCollisionObjects() == 0);

    // Delete all the collision shapes. This can be done at any time.
    const size_t ncs = m_collisionShapes.size();
    for (size_t i = 0; i < ncs; ++i) { delete m_collisionShapes[i]; }
}

void tgWorldBulletPhysicsImpl::removeConstraints()
{
    const size_t nc = m_pDynamicsWorld->getNumConstraints();
    for (int i = nc - 1; i >= 0; --i)
    {
        btTypedConstraint * const pTypedConstraint = m_pDynamicsWorld->getConstraint(i);

        // Remove the constraint from the dynamics world
        m_pDynamicsWorld->removeConstraint(pTypedConstraint);
    }
    // All constraints have been removed and deleted
    assert(m_pDynamicsWorld->getNumConstraints() == 0);
}

/**
 * Create and return a new instance of a btSoftRigidDynamicsWorld.
 * @return a pointer to a new instance of a btSoftRigidDynamicsWorld
 */
btSoftRigidDynamicsWorld* tgWorldBulletPhysicsImpl::createDynamicsWorld() const
{    
    btSoftRigidDynamicsWorld * const result =
        new btSoftRigidDynamicsWorld(&m_pIntermediateBuildProducts->dispatcher,
                &m_pIntermediateBuildProducts->broadphase,
                &m_pIntermediateBuildProducts->solver, 
                &m_pIntermediateBuildProducts->collisionConfiguration);

    return result;
}

/**
 * Create and return a new instance of a btRigidBody for the ground shape.
 * This will be added to the btSoftRigidDynamicsWorld.
 */
btRigidBody* tgWorldBulletPhysicsImpl::createGroundRigidBody()
{
    const btScalar mass = 0.0;

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -0.5, 0));

    // Using motionstate is recommended
    // It provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* const pMotionState =
        new btDefaultMotionState(groundTransform);

    /* The Problem Area */
    //TODO: Change to be polymorphic
    //const btVector3 groundDimensions(btScalar(500.0), btScalar(0.5), btScalar(500.0));
    //btBoxShape* const pGroundShape = new btBoxShape(groundDimensions);

    // Determine the angle of the ground in radians. All 0 is flat         
    
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgHillyGround::Config groundConfig(btVector3(yaw, pitch, roll));
    tgHillyGround *hillyGround = new tgHillyGround(groundConfig);
    btCollisionShape *const pGroundShape = hillyGround->hillyCollisionShape();
    /**/

    addCollisionShape(pGroundShape);

    const btVector3 localInertia(0, 0, 0);

    btRigidBody::btRigidBodyConstructionInfo const rbInfo(mass, pMotionState, pGroundShape, localInertia);

    btRigidBody* const pGroundRigidBody = new btRigidBody(rbInfo);

    return pGroundRigidBody;
}


void tgWorldBulletPhysicsImpl::step(double dt)
{
    // Precondition
    assert(dt > 0.0);

    const btScalar timeStep = dt;
    const int maxSubSteps = 1;
    const btScalar fixedTimeStep = dt;
    m_pDynamicsWorld->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);

    // Postcondition
    assert(invariant());
}

void tgWorldBulletPhysicsImpl::addCollisionShape(btCollisionShape* pShape)
{
    if (pShape)
    {
        std::cout << "Pushed shape: " << pShape->getName() << std::endl;
        m_collisionShapes.push_back(pShape);
    }

    // Postcondition
    assert(invariant());
}

void tgWorldBulletPhysicsImpl::addConstraint(btTypedConstraint* pConstraint)
{
      assert(invariant());

      if (pConstraint)
      {
            m_constraints.push_back(pConstraint);
            m_pDynamicsWorld->addConstraint(pConstraint);
      }

      // Postcondition
      assert(invariant());
}

bool tgWorldBulletPhysicsImpl::invariant() const
{
    return (m_pDynamicsWorld != 0);
}

