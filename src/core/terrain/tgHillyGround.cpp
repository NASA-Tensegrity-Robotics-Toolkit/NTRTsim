/**
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
 * @file tgHillyGround.cpp
 * @brief Contains the implementation of class tgHillyGround
 * @author Steven Lessard
 * $Id$
 */

//This Module
#include "tgHillyGround.h"

//Bullet Physics
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btTransform.h"

// The C++ Standard Library
#include <cassert>
#include <iostream>

tgHillyGround::Config::Config(btVector3 eulerAngles,
        double friction,
        double restitution,
        btVector3 size,
        btVector3 origin,
        std::size_t nx,
        std::size_t ny,
        double margin,
        double triangleSize,
        double waveHeight,
        double offset) :
    m_eulerAngles(eulerAngles),
    m_friction(friction),
    m_restitution(restitution),
    m_size(size),
    m_origin(origin),
    m_nx(nx),
    m_ny(ny),
    m_margin(margin),
    m_triangleSize(triangleSize),
    m_waveHeight(waveHeight),
    m_offset(offset)
{
    assert((m_friction >= 0.0) && (m_friction <= 1.0));
    assert((m_restitution >= 0.0) && (m_restitution <= 1.0));
    assert((m_size[0] >= 0.0) && (m_size[1] >= 0.0) && (m_size[2] >= 0.0));
    assert(m_nx > 0);
    assert(m_ny > 0);
    assert(m_margin >= 0.0);
    assert(m_triangleSize >= 0.0);
    assert(m_waveHeight >= 0.0);
    assert(m_offset >= 0.0);
}

tgHillyGround::tgHillyGround() :
    m_config(Config())
{
    // @todo make constructor aux to avoid repeated code
    pGroundShape = hillyCollisionShape();
}

tgHillyGround::tgHillyGround(const tgHillyGround::Config& config) :
    m_config(config)
{
    pGroundShape = hillyCollisionShape();
}

tgHillyGround::~tgHillyGround()
{
    delete m_pMesh;
    delete[] m_pIndices;
    delete[] m_vertices;
}

btRigidBody* tgHillyGround::getGroundRigidBody() const
{
        std::cout << "Hilly ground " << std::endl;
    const btScalar mass = 0.0;

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(m_config.m_origin);

    btQuaternion orientation;
    orientation.setEuler(m_config.m_eulerAngles[0], // Yaw
                         m_config.m_eulerAngles[1], // Pitch
                         m_config.m_eulerAngles[2]); // Roll
    groundTransform.setRotation(orientation);

    // Using motionstate is recommended
    // It provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* const pMotionState =
        new btDefaultMotionState(groundTransform);

    const btVector3 localInertia(0, 0, 0);

    btRigidBody::btRigidBodyConstructionInfo const rbInfo(mass, pMotionState, pGroundShape, localInertia);

    btRigidBody* const pGroundBody = new btRigidBody(rbInfo);

    assert(pGroundBody);
    return pGroundBody;
}  

btCollisionShape* tgHillyGround::hillyCollisionShape() {
    btCollisionShape * pShape = 0;
    // The number of vertices in the mesh
    // Hill Paramenters: Subject to Change
    const std::size_t vertexCount = m_config.m_nx * m_config.m_ny;

    if (vertexCount > 0) {
        // The number of triangles in the mesh
        const std::size_t triangleCount = 2 * (m_config.m_nx - 1) * (m_config.m_ny - 1);

        // A flattened array of all vertices in the mesh
        m_vertices = new btVector3[vertexCount];

        // Supplied by the derived class
        setVertices(m_vertices);
        // A flattened array of indices for each corner of each triangle
        m_pIndices = new int[triangleCount * 3];

        // Supplied by the derived class
        setIndices(m_pIndices);

        // Create the mesh object
        m_pMesh = createMesh(triangleCount, m_pIndices, vertexCount, m_vertices);

        // Create the shape object
        pShape = createShape(m_pMesh);

        // Set the margin
        pShape->setMargin(m_config.m_margin);
        // DO NOT deallocate vertices, indices or pMesh until simulation is over!
        // The shape owns them, but will not delete them
    }

    assert(pShape);
    return pShape; 
}

btTriangleIndexVertexArray *tgHillyGround::createMesh(std::size_t triangleCount, int indices[], std::size_t vertexCount, btVector3 vertices[]) {
    const int vertexStride = sizeof(btVector3);
    const int indexStride = 3 * sizeof(int);

    btTriangleIndexVertexArray* const pMesh = 
        new btTriangleIndexVertexArray(triangleCount,
                indices,
                indexStride,
                vertexCount,
                (btScalar*) &vertices[0].x(),
                vertexStride);
    return pMesh;
}

btCollisionShape *tgHillyGround::createShape(btTriangleIndexVertexArray *pMesh) {
    const bool useQuantizedAabbCompression = true;
    btCollisionShape *const pShape = 
        new btBvhTriangleMeshShape(pMesh, useQuantizedAabbCompression);
    return pShape;
}

void tgHillyGround::setVertices(btVector3 vertices[]) {
    for (std::size_t i = 0; i < m_config.m_nx; i++)
    {
        for (std::size_t j = 0; j < m_config.m_ny; j++)
        {
            const btScalar x = (i - (m_config.m_nx * 0.5)) * m_config.m_triangleSize;
            const btScalar y = (m_config.m_waveHeight * sin((double)i) * cos((double)j) +
                    m_config.m_offset);
            const btScalar z = (j - (m_config.m_ny * 0.5)) * m_config.m_triangleSize;
            vertices[i + (j * m_config.m_nx)].setValue(x, y, z);
        }
    }
}

void tgHillyGround::setIndices(int indices[]) {
    int index = 0;
    for (std::size_t i = 0; i < m_config.m_nx - 1; i++)
    {
        for (std::size_t j = 0; j < m_config.m_ny - 1; j++)
        {
            indices[index++] = (j       * m_config.m_nx) + i;
            indices[index++] = (j       * m_config.m_nx) + i + 1;
            indices[index++] = ((j + 1) * m_config.m_nx) + i + 1;

            indices[index++] = (j       * m_config.m_nx) + i;
            indices[index++] = ((j + 1) * m_config.m_nx) + i + 1;
            indices[index++] = ((j + 1) * m_config.m_nx) + i;
        }
    }
}

