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
#include "core/terrain/tgBulletGround.h"
// The Bullet Physics library
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btScalar.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
//#include "LinearMath/btiDebugDraw.h"
#include <iostream>

/**
 * Helper class to bundle objects that have the same life cycle, so they can be
 * constructed and destructed together.
 */
class IntermediateBuildProducts
{
public:
  IntermediateBuildProducts() : dispatcher(&collisionConfiguration)
  {
  }
  btSoftBodyRigidBodyCollisionConfiguration collisionConfiguration;
  btCollisionDispatcher dispatcher;
  btDbvtBroadphase broadphase;
  btSequentialImpulseConstraintSolver solver;
};

tgWorldBulletPhysicsImpl::tgWorldBulletPhysicsImpl(const tgWorld::Config& config,
                                                    tgBulletGround* ground) :
    tgWorldImpl(config, ground),
    m_pIntermediateBuildProducts(new IntermediateBuildProducts),
    m_pDynamicsWorld(createDynamicsWorld())
{
  // Gravitational acceleration is down on the Y axis
  const btVector3 gravityVector(0, -config.gravity, 0);
  m_pDynamicsWorld->setGravity(gravityVector);
    
    
    // Create and add the ground rigid body
    #if (0)
    btRigidBody * const pGroundRigidBody = createGroundRigidBody();
    m_pDynamicsWorld->addRigidBody(pGroundRigidBody);
    #else
    m_pDynamicsWorld->addRigidBody(ground->getGroundRigidBody());
    #endif
    
    
    // create and add duct using 8 boxes
    
    // vertical walls
    const btVector3 boxDimensions1(btScalar(0.1), btScalar(40), btScalar(0.5));
    const btVector3 Origin1(0, 40, -17);
    
    const btVector3 boxDimensions2(btScalar(17), btScalar(40), btScalar(0.5));
    const btVector3 Origin2(0, 40, 17);
    
    const btVector3 boxDimensions3(btScalar(0.5), btScalar(40), btScalar(17));
    const btVector3 Origin3(-17, 40, 0);
    
    const btVector3 boxDimensions4(btScalar(0.5), btScalar(23), btScalar(17));
    const btVector3 Origin4(17, 23, 0);
    
    //horizontal walls
    const btVector3 boxDimensions5(btScalar(30), btScalar(0.1), btScalar(0.5));
    const btVector3 Origin5(47, 63, -17);
    
    const btVector3 boxDimensions6(btScalar(30), btScalar(17), btScalar(0.5));
    const btVector3 Origin6(47, 63, 17);
    
    const btVector3 boxDimensions7(btScalar(47), btScalar(0.5), btScalar(17));
    const btVector3 Origin7(30, 80, 0);
    
    const btVector3 boxDimensions8(btScalar(30), btScalar(0.5), btScalar(17));
    const btVector3 Origin8(47, 46, 0);


    
    //create boxes
    //btRigidBody * const pBoxRigidBody1 = createBoxRigidBody(boxDimensions1, Origin1);
    btRigidBody * const pBoxRigidBody2 = createBoxRigidBody(boxDimensions2, Origin2);
    btRigidBody * const pBoxRigidBody3 = createBoxRigidBody(boxDimensions3, Origin3);
    btRigidBody * const pBoxRigidBody4 = createBoxRigidBody(boxDimensions4, Origin4);
    btRigidBody * const pBoxRigidBody5 = createBoxRigidBody(boxDimensions5, Origin5);
    btRigidBody * const pBoxRigidBody6 = createBoxRigidBody(boxDimensions6, Origin6);
    btRigidBody * const pBoxRigidBody7 = createBoxRigidBody(boxDimensions7, Origin7);
    btRigidBody * const pBoxRigidBody8 = createBoxRigidBody(boxDimensions8, Origin8);
    
    // add boxes to world
    // why does duct disappear when simulation is reset???
    //m_pDynamicsWorld->addRigidBody(pBoxRigidBody1);
    m_pDynamicsWorld->addRigidBody(pBoxRigidBody2);
    m_pDynamicsWorld->addRigidBody(pBoxRigidBody3);
    m_pDynamicsWorld->addRigidBody(pBoxRigidBody4);
    m_pDynamicsWorld->addRigidBody(pBoxRigidBody5);
    m_pDynamicsWorld->addRigidBody(pBoxRigidBody6);
    m_pDynamicsWorld->addRigidBody(pBoxRigidBody7);
    m_pDynamicsWorld->addRigidBody(pBoxRigidBody8);
    
    
    
    // test
    
    btRigidBody * const InvisibleBox = createInvisibleBox(boxDimensions1, Origin1);
    m_pDynamicsWorld->addRigidBody(InvisibleBox);
    
    // invisible sphere test
//    btBoxShape* const pBoxTemp = new btBoxShape(boxDimensions1);
//    btGhostObject* ghostObject = new btGhostObject();
//    ghostObject->setCollisionShape(pBoxTemp);
//    ghostObject->setWorldTransform(btTransform(btQuaternion(0,0,0,1),btVector3(0,15,0)));
//    m_pDynamicsWorld->addCollisionObject(ghostObject);
    
    
//    // add sphere for fun
//    btRigidBody * const pSphereRigidBody = createSphereRigidBody();
//    m_pDynamicsWorld->addRigidBody(pSphereRigidBody);


  // Postcondition
  assert(invariant());
}

tgWorldBulletPhysicsImpl::~tgWorldBulletPhysicsImpl()
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

  delete m_pDynamicsWorld;

  // Delete the intermediate build products, which are now orphaned
  delete m_pIntermediateBuildProducts;
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

    const btVector3 groundDimensions(btScalar(500.0), btScalar(0.5), btScalar(500.0));
    btBoxShape* const pGroundShape = new btBoxShape(groundDimensions);
    
    addCollisionShape(pGroundShape);
    std::cout << "add ground" << std::endl;
    
    const btVector3 localInertia(0, 0, 0);

    btRigidBody::btRigidBodyConstructionInfo const rbInfo(mass, pMotionState, pGroundShape, localInertia);

    btRigidBody* const pGroundRigidBody = new btRigidBody(rbInfo);

    return pGroundRigidBody;
}

// create boxes to make duct
btRigidBody* tgWorldBulletPhysicsImpl::createBoxRigidBody(btVector3 boxDimensions, btVector3 Origin)
{
    std::cout << "create box" << std::endl;
    const btScalar mass = 0.0;
    
    btTransform boxTransform;
    boxTransform.setIdentity();
    boxTransform.setOrigin(Origin);
    
    // Using motionstate is recommended
    // It provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* const pMotionState =
    new btDefaultMotionState(boxTransform);
    
        btBoxShape* const pBoxShape = new btBoxShape(boxDimensions);
    
    addCollisionShape(pBoxShape);
    
    const btVector3 localInertia(0, 0, 0);
    
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, pMotionState, pBoxShape, localInertia);
    rbInfo.m_friction = btScalar(0.4);
    
    
    btRigidBody* const pBoxRigidBody = new btRigidBody(rbInfo);
    
    return pBoxRigidBody;
}

btRigidBody* tgWorldBulletPhysicsImpl::createInvisibleBox(const btVector3 boxDimensions, const btVector3 Origin) {
	
	// Set the initial position of the object
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(Origin);
    
	btDefaultMotionState *MotionState = new btDefaultMotionState(Transform);
    
	// Create the shape
	//btVector3 HalfExtents(TScale.X * 0.5f, TScale.Y * 0.5f, TScale.Z * 0.5f);
	btCollisionShape *Shape = new btBoxShape(boxDimensions);
    
	// Add mass
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(0.0f, LocalInertia);
    
	// Create the rigid body object
	btRigidBody *RigidBody = new btRigidBody(0.0f, MotionState, Shape, LocalInertia);
    
	// Store a pointer to the irrlicht node so we can update it later
	RigidBody->setUserPointer(NULL);
    
    //m_collisionShapes.push_back(RigidBody);
    
    return RigidBody;
}


// add random sphere for fun
//btRigidBody* tgWorldBulletPhysicsImpl::createSphereRigidBody()
//{
//    const btScalar mass = 0.0;
//    
//    btTransform sphereTransform;
//    sphereTransform.setIdentity();
//    sphereTransform.setOrigin(btVector3(0,10,0));
//    
//    // Using motionstate is recommended
//    // It provides interpolation capabilities, and only synchronizes 'active' objects
//    btDefaultMotionState* const pMotionState =
//    new btDefaultMotionState(sphereTransform);
//    
//    btSphereShape* const pSphereShape = new btSphereShape(5);
//    
//    addCollisionShape(pSphereShape);
//    
//    const btVector3 localInertia(0, 0, 0);
//    
//    btRigidBody::btRigidBodyConstructionInfo const rbInfo(mass, pMotionState, pSphereShape, localInertia);
//    
//    btRigidBody* const pSphereRigidBody = new btRigidBody(rbInfo);
//    
//    return pSphereRigidBody;
//}





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
        m_collisionShapes.push_back(pShape);
      }

      // Postcondition
      assert(invariant());
}

bool tgWorldBulletPhysicsImpl::invariant() const
{
        return (m_pDynamicsWorld != 0);
}
